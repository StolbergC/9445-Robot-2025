from math import pi
from typing import Callable
from rev import SparkMax, SparkMaxConfig, SparkBase, EncoderConfig

from commands2 import (
    InstantCommand,
    RepeatCommand,
    Subsystem,
    RunCommand,
    WrapperCommand,
)

from ntcore import NetworkTable, NetworkTableInstance, EventFlags, Event, ValueEventData

from wpimath.controller import ProfiledPIDController, ArmFeedforward
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d

from wpilib import DutyCycleEncoder


class Wrist(Subsystem):
    """
    YOU MUST SET THE GET CLAW DISTANCE AND SAFE DISTANCE AFTER CONSTRUCTION.
    THIS IS INTENDED TO ALLOW THE CLAW TO BE CONSTRUCTED AFTER AND THEN PASS THE VALUES IN
    """

    get_claw_distance: Callable[[], float]
    safe_claw_distance: float = 1

    def __init__(self):
        super().__init__()
        self.motor = SparkMax(20, SparkBase.MotorType.kBrushless)
        self.encoder = self.motor.getAbsoluteEncoder()
        self.motor_config = (
            SparkMaxConfig()
            .smartCurrentLimit(30)
            .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        )

        self.motor_config.absoluteEncoder.zeroOffset(0.281).zeroCentered(
            True
        ).positionConversionFactor(360).velocityConversionFactor(360)

        self.motor.configure(
            self.motor_config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters,
        )

        self.pid = ProfiledPIDController(
            17, 0, 0, TrapezoidProfile.Constraints(2 * pi, 100 * pi)
        )

        self.feedforward = ArmFeedforward(0, 0.25, 0, 0)

        self.nettable = NetworkTableInstance.getDefault().getTable("000Wrist")

        def nettable_updater(_nt: NetworkTable, key: str, ev: Event) -> None:
            if isinstance(data := ev.data, ValueEventData):
                if key == "PID/p":
                    self.pid.setP(data.value.value())
                elif key == "PID/i":
                    self.pid.setI(data.value.value())
                elif key == "PID/d":
                    self.pid.setD(data.value.value())
                elif key == "Feedforward/kS":
                    self.feedforward = ArmFeedforward(
                        data.value.value(),
                        self.feedforward.getKg(),
                        self.feedforward.getKv(),
                        self.feedforward.getKa(),
                    )
                elif key == "Feedforward/kG":
                    self.feedforward = ArmFeedforward(
                        self.feedforward.getKs(),
                        data.value.value(),
                        self.feedforward.getKv(),
                        self.feedforward.getKa(),
                    )
                elif key == "Feedforward/kV":
                    self.feedforward = ArmFeedforward(
                        self.feedforward.getKs(),
                        self.feedforward.getKg(),
                        data.value.value(),
                        self.feedforward.getKa(),
                    )
                elif key == "Feedforward/kA":
                    self.feedforward = ArmFeedforward(
                        self.feedforward.getKs(),
                        self.feedforward.getKg(),
                        self.feedforward.getKv(),
                        data.value.value(),
                    )

        self.nettable.addListener("PID/p", EventFlags.kValueAll, nettable_updater)
        self.nettable.addListener("PID/i", EventFlags.kValueAll, nettable_updater)
        self.nettable.addListener("PID/d", EventFlags.kValueAll, nettable_updater)
        self.nettable.addListener(
            "Feedforward/kS", EventFlags.kValueAll, nettable_updater
        )
        self.nettable.addListener(
            "Feedforward/kG", EventFlags.kValueAll, nettable_updater
        )
        self.nettable.addListener(
            "Feedforward/kV", EventFlags.kValueAll, nettable_updater
        )
        self.nettable.addListener(
            "Feedforward/kA", EventFlags.kValueAll, nettable_updater
        )

        self.nettable.putNumber("PID/p", self.pid.getP())
        self.nettable.putNumber("PID/i", self.pid.getI())
        self.nettable.putNumber("PID/d", self.pid.getD())
        self.nettable.putNumber("Feedforward/kS", self.feedforward.getKs())
        self.nettable.putNumber("Feedforward/kG", self.feedforward.getKg())
        self.nettable.putNumber("Feedforward/kV", self.feedforward.getKv())
        self.nettable.putNumber("Feedforward/kA", self.feedforward.getKa())

        self.setpoint: Rotation2d = Rotation2d.fromDegrees(0)

    def periodic(self) -> None:
        self.pid.calculate(self.get_angle().radians())
        self.nettable.putNumber("State/angle (deg)", self.get_angle().degrees())
        self.nettable.putNumber(
            "State/velocity (rad p s)", self.get_velocity().radians()
        )
        self.nettable.putNumber(
            "Commanded/speed (rad p s)", self.pid.getSetpoint().velocity
        )
        self.nettable.putNumber(
            "Commanded/speed goal (rad p s)", self.pid.getGoal().velocity
        )
        self.nettable.putNumber(
            "Commanded/angle pid (rads)", self.pid.getSetpoint().position
        )
        if (c := self.getCurrentCommand()) is not None:
            self.nettable.putString("Running Command", c.getName())
        else:
            self.nettable.putString("Running Command", "None")
        return super().periodic()

    def stop(self) -> InstantCommand:
        return InstantCommand(lambda: self.motor.set(0))

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.encoder.getPosition())

    def get_velocity(self) -> Rotation2d:
        """rotation2d/s"""
        return Rotation2d.fromDegrees(self.encoder.getVelocity())

    def follow_angle(self, angle: Callable[[], Rotation2d] | None = None) -> RunCommand:
        return RunCommand(
            lambda: self.set_state(angle() if angle is not None else self.setpoint),
            self,
        )

    def set_state(self, angle: Rotation2d) -> None:
        self.nettable.putNumber("Commanded/angle (deg)", angle.degrees())
        if (
            abs(self.get_claw_distance() - self.safe_claw_distance)
            > 1  # TODO: This might be stupid. The claw has to be in the center of its range
            and angle.degrees() >= 75  # this means that we are going up
        ):
            self.nettable.putBoolean("Safety/Waiting on Claw", True)
            return
        self.nettable.putBoolean("Safety/Waiting on Claw", False)
        if angle.degrees() < -50:  # more ground pointing
            angle = Rotation2d.fromDegrees(-50)
        elif angle.degrees() > 90:  # more sky pointing
            angle = Rotation2d.fromDegrees(90)
        self.setpoint = angle
        self.pid.setGoal(angle.radians())
        volts = self.pid.calculate(
            self.get_angle().radians(), angle.radians()
        ) + self.feedforward.calculate(
            self.get_angle().radians(),
            self.get_velocity().radians(),
            self.pid.getSetpoint().velocity,
        )

        self.nettable.putNumber("State/Speed (V)", volts)
        self.motor.setVoltage(volts)

    def run_angle(self, angle: Rotation2d) -> WrapperCommand:
        return (
            RepeatCommand(InstantCommand(lambda: self.set_state(angle), self))
            .onlyWhile(lambda: abs(angle.degrees() - self.get_angle().degrees()) > 1)
            .andThen(self.stop())
            .withName(f"Set Angle {angle.degrees()} (deg)")
        )

    def angle_intake(self) -> WrapperCommand:
        return self.run_angle(Rotation2d.fromDegrees(55)).withName("Intake")

    def angle_score(self) -> WrapperCommand:
        return self.run_angle(Rotation2d.fromDegrees(-20)).withName("Score")

    def angle_zero(self) -> WrapperCommand:
        return self.run_angle(Rotation2d.fromDegrees(10)).withName("Horizontal")

    def angle_full_up(self) -> WrapperCommand:
        return self.run_angle(Rotation2d.fromDegrees(90)).withName("Max Angle")

    def command_intake(self) -> InstantCommand:
        def do_it():
            self.setpoint = Rotation2d.fromDegrees(55)

        return InstantCommand(do_it)

    def command_score(self) -> InstantCommand:
        def do_it():
            self.setpoint = Rotation2d.fromDegrees(-18.5)

        return InstantCommand(do_it)

    def command_zero(self) -> InstantCommand:
        def do_it():
            self.setpoint = Rotation2d.fromDegrees(10)

        return InstantCommand(do_it)

    def manual_control(self, power: Callable[[], float]) -> RunCommand:
        """This should only be used in test mode for the pit to reset the robot"""
        return RunCommand(lambda: self.motor.set(power()), self)
