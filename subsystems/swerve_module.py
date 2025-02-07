import math
import time

from rev import SparkBaseConfig, SparkMax, SparkLowLevel, SparkMaxConfig, SparkBase

from phoenix6.hardware import CANcoder
from phoenix6.configs.config_groups import MagnetSensorConfigs
from phoenix6.signals import SensorDirectionValue

from commands2 import Subsystem

from ntcore import NetworkTableInstance, EventFlags, Event, ValueEventData

from wpilib import RobotBase
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.units import (
    inchesToMeters,
    meters_per_second,
    meters_per_second_squared,
    feetToMeters,
)
from wpimath.geometry import Rotation2d
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile

module_offsets = {
    "fl": 0.08544921875,
    "fr": -0.37451171875,
    "bl": 0.15283203125,
    "br": -0.140625,
}


class SwerveModule(Subsystem):
    def __init__(
        self,
        name: str,
        drive_id: int,
        turn_id: int,
        cancoder_id: int,
        drive_inverted: bool,
        turn_inverted: bool,
        max_velocity: meters_per_second,
        max_accel: meters_per_second_squared,
    ):
        self.is_real = RobotBase.isReal()

        # cancoder
        self.cancoder = CANcoder(cancoder_id)
        self.cancoder.configurator.apply(
            MagnetSensorConfigs()
            .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
            .with_absolute_sensor_discontinuity_point(0.5)
            .with_magnet_offset(module_offsets[name])
        )

        self.cancoder.optimize_bus_utilization()
        self.cancoder.get_absolute_position().set_update_frequency(100)

        time.sleep(0.1)

        self.name = name
        self.setName(name)

        # drive
        self.drive_motor = SparkMax(drive_id, SparkLowLevel.MotorType.kBrushless)
        self.drive_encoder = self.drive_motor.getEncoder()
        self.drive_pid = ProfiledPIDController(
            0.2, 0, 0, TrapezoidProfile.Constraints(max_velocity, max_accel)
        )

        self.drive_motor_config = SparkMaxConfig()
        self.drive_motor_config.inverted(drive_inverted).smartCurrentLimit(30, 10)

        self.drive_encoder.setPosition(0)

        self.drive_motor_config.signals.absoluteEncoderPositionAlwaysOn(
            False
        ).analogVoltageAlwaysOn(False)

        self.drive_motor.configure(
            self.drive_motor_config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        # turn
        self.turn_motor = SparkMax(turn_id, SparkLowLevel.MotorType.kBrushless)
        """
        the range of error for this controller is [-0.25, 0.25] 
        """
        self.turn_pid = ProfiledPIDController(
            0.01, 0, 0, TrapezoidProfile.Constraints(360, 3600)
        )
        self.turn_motor_config = SparkMaxConfig()
        self.turn_motor_config.inverted(turn_inverted).smartCurrentLimit(20, 10)

        self.turn_pid.enableContinuousInput(-180, 180)

        self.turn_motor.configure(
            self.turn_motor_config,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters,
        )

        # networktables
        self.nettable = NetworkTableInstance.getDefault().getTable(
            f"00Swerve/{self.name}"
        )

        self.nettable.addListener("driveP", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener("driveI", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener("driveD", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener("turnD", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener("turnI", EventFlags.kValueAll, self._nt_pid_listener)
        self.nettable.addListener("turnP", EventFlags.kValueAll, self._nt_pid_listener)

        self.nettable.putNumber("driveP", self.drive_pid.getP())
        self.nettable.putNumber("driveI", self.drive_pid.getI())
        self.nettable.putNumber("driveD", self.drive_pid.getD())
        self.nettable.putNumber("turnP", self.turn_pid.getP())
        self.nettable.putNumber("turnI", self.turn_pid.getI())
        self.nettable.putNumber("turnD", self.turn_pid.getD())

        self.commanded_state = SwerveModuleState(0, Rotation2d(0))

        self.last_position: float = 0

        if not RobotBase.isReal():
            self.drive_pid.setPID(0, 0, 0)
            self.turn_pid.setPID(0, 0, 0)

    def periodic(self) -> None:

        self.nettable.putNumber("State/velocity (mps)", self.get_vel())
        self.nettable.putNumber("State/angle (deg)", self.get_angle().degrees())
        self.nettable.putNumber(
            "State/cancoder position (rotation)",
            self.cancoder.get_absolute_position().value_as_double,
        )

        self.nettable.putNumber("State/drive position", self.get_distance())

    def get_vel(self) -> float:
        """
        return the velocity in meters per second

        circumfrence of a circle is 2 * pi * r.
        The wheel radius of the mk4i is 2in.
        The gear ratio of the mk4i is 8.14:1
        """
        return (
            self.drive_encoder.getVelocity()
            * 2
            * math.pi
            * inchesToMeters(2)
            * 8.14
            / 4096
        )

    def get_distance(self) -> float:
        """return the distance driven by the swerve module since powered on"""
        return feetToMeters(self.drive_encoder.getPosition() / 8.14)

    def get_angle(self) -> Rotation2d:
        """return the angle of the swerve module as a Rotation2d
        This does not work
        """
        p = self.cancoder.get_absolute_position()
        return Rotation2d.fromDegrees(
            (p.value_as_double if p.is_all_good() else self.last_position) * 360
        )

    def get_state(self) -> SwerveModuleState:
        """return the velocity and angle of the swerve module"""
        if not self.is_real:
            return self.commanded_state
        return SwerveModuleState(self.get_vel(), self.get_angle())

    def get_position(self) -> SwerveModulePosition:
        """get the distance driven and angle of the module"""
        return SwerveModulePosition(self.get_distance(), self.get_angle())

    def set_max_vel(self, max_vel: float) -> None:
        self.drive_pid.setConstraints(
            TrapezoidProfile.Constraints(max_vel, max_vel * 5)
        )

    def set_drive_idle(self, coast: bool) -> None:
        self.drive_motor_config.setIdleMode(
            SparkBaseConfig.IdleMode.kCoast
            if not coast
            else SparkBaseConfig.IdleMode.kBrake
        )
        self._configure_drive()

    def set_turn_idle(self, coast: bool) -> None:
        self.turn_motor_config.setIdleMode(
            SparkBaseConfig.IdleMode.kCoast
            if not coast
            else SparkBaseConfig.IdleMode.kBrake
        )
        self._configure_turn()

    def set_state(self, commanded_state: SwerveModuleState) -> None:
        """command the swerve module to an angle and speed"""
        # optimize the new state
        # this just mutates commanded_state in place
        self.commanded_state = commanded_state
        commanded_state.optimize(self.get_angle())

        self.nettable.putNumber(
            "Commanded/Angle (deg)", commanded_state.angle.degrees()
        )

        self.nettable.putNumber(
            "State/turn error",
            -self.get_angle().degrees() + commanded_state.angle.degrees(),
        )

        turn_speed = self.turn_pid.calculate(
            self.get_angle().degrees(), commanded_state.angle.degrees()
        )
        self.nettable.putNumber("State/Turn Speed (%)", turn_speed)
        turn_speed = 1 if turn_speed > 1 else -1 if turn_speed < -1 else turn_speed
        self.turn_motor.set(turn_speed)

        """
        cosine optimization - make the wheel slower when pointed the wrong direction
        note that there in no abs over cos because cos(-x) == cos(x)
        """
        cos_optimizer = (commanded_state.angle - self.get_angle()).cos()
        self.nettable.putNumber("Commanded/Speed", commanded_state.speed)
        self.nettable.putNumber(
            "Commanded/Optimized Speed", commanded_state.speed * cos_optimizer
        )

        drive_speed = self.drive_pid.calculate(
            self.get_vel(), commanded_state.speed * cos_optimizer
        )
        drive_speed = 1 if drive_speed > 1 else -1 if drive_speed < -1 else drive_speed
        self.drive_motor.set(drive_speed)
        self.nettable.putNumber("State/Out Drive Speed (%)", drive_speed)

    def _rotation2d_to_rotations(self, angle: Rotation2d) -> float:
        return angle.degrees() / 360

    def _nt_pid_listener(self, _nt, key: str, event: Event):
        try:
            var = key[-1]
            if isinstance((v := event.data), ValueEventData):
                if key.startswith("drive"):
                    if var == "P":
                        self.drive_pid.setP(v.value.value())
                    elif var == "I":
                        self.drive_pid.setI(v.value.value())
                    elif var == "D":
                        self.drive_pid.setD(v.value.value())
                elif key.startswith("turn"):
                    if var == "P":
                        self.turn_pid.setP(v.value.value())
                    elif var == "I":
                        self.turn_pid.setI(v.value.value())
                    elif var == "D":
                        self.turn_pid.setD(v.value.value())
                else:
                    print(f"failed at {key}")
        except Exception:
            pass

    def _configure_drive(self) -> None:
        self.drive_motor.configure(
            self.drive_motor_config,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters,
        )

    def _configure_turn(self) -> None:
        self.turn_motor.configure(
            self.turn_motor_config,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters,
        )
