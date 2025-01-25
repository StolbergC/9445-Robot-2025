from rev import SparkMax, SparkMaxConfig, SparkBase
from phoenix6.hardware import CANcoder
from phoenix6.configs.config_groups import MagnetSensorConfigs
from phoenix6.signals import SensorDirectionValue

from commands2 import Subsystem, RunCommand

from ntcore import NetworkTable, NetworkTableInstance, EventFlags, Event, ValueEventData

from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d


class Wrist(Subsystem):
    def __init__(self):
        self.motor = SparkMax(20, SparkBase.MotorType.kBrushless)
        self.motor_config = (
            SparkMaxConfig()
            .smartCurrentLimit(20)
            .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        )

        self.motor.configure(
            self.motor_config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters,
        )

        self.cancoder = CANcoder(21)
        self.cancoder.configurator.apply(
            MagnetSensorConfigs()
            .with_absolute_sensor_discontinuity_point(0.5)
            .with_magnet_offset(0)  # change when finding the actual offset
            .with_sensor_direction(
                SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
            )  # the motor is opposite to the encoder
        )

        self.pid = ProfiledPIDController(
            0.01, 0, 0, TrapezoidProfile.Constraints(90, 1800)
        )

        self.nettable = NetworkTableInstance().getDefault().getTable("wrist")

        def nettable_updater(_nt: NetworkTable, key: str, ev: Event) -> None:
            if isinstance(data := ev.data, ValueEventData):
                if key == "PID/p":
                    self.pid.setP(data.value.value())
                elif key == "PID/i":
                    self.pid.setI(data.value.value())
                elif key == "PID/d":
                    self.pid.setD(data.value.value())

        self.nettable.addListener("PID/p", EventFlags.kValueAll, nettable_updater)
        self.nettable.addListener("PID/i", EventFlags.kValueAll, nettable_updater)
        self.nettable.addListener("PID/d", EventFlags.kValueAll, nettable_updater)

        self.nettable.putNumber("PID/p", self.pid.getP())
        self.nettable.putNumber("PID/i", self.pid.getI())
        self.nettable.putNumber("PID/d", self.pid.getD())

    def periodic(self) -> None:
        self.nettable.putNumber("State/angle (deg)", self.get_angle().degrees())
        return super().periodic()

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(
            self.cancoder.get_absolute_position().value_as_double * 360
        )

    def set_state(self, angle: Rotation2d) -> None:
        if angle.degrees() < -90:
            angle = Rotation2d.fromDegrees(-90)
        elif angle.degrees() > 90:
            angle = Rotation2d.fromDegrees(90)
        speed = self.pid.calculate(self.get_angle().degrees(), angle.degrees())
        speed = 1 if speed > 1 else -1 if speed < -1 else speed
        self.motor.set(speed)

    def run_angle(self, angle: Rotation2d) -> RunCommand:
        return RunCommand(lambda: self.set_state(angle), self)

    def angle_intake(self) -> RunCommand:
        return self.run_angle(Rotation2d.fromDegrees(55))

    def angle_score(self) -> RunCommand:
        return self.run_angle(Rotation2d.fromDegrees(-35))
