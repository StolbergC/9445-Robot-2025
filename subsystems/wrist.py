from math import pi
from typing import Callable
from rev import SparkMax, SparkMaxConfig, SparkBase

from commands2 import Subsystem, RunCommand, WrapperCommand

from ntcore import NetworkTable, NetworkTableInstance, EventFlags, Event, ValueEventData

from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d

from wpilib import DutyCycleEncoder


class Wrist(Subsystem):
    """
    YOU MUST SET THE GET CLAW DISTANCE AND SAFE DISTANCE AFTER CONSTRUCTION.
    THIS IS INTENDED TO ALLOW THE CLAW TO BE CONSTRUCTED AFTER AND THEN PASS THE VALUES IN
    """

    get_claw_distance: Callable[[], float]
    safe_claw_distance: float

    def __init__(self):
        super().__init__()
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

        self.encoder = DutyCycleEncoder(4, 180, 90)

        self.pid = ProfiledPIDController(
            0.01, 0, 0, TrapezoidProfile.Constraints(pi / 2, 20 * pi)
        )

        self.nettable = NetworkTableInstance.getDefault().getTable("000Wrist")

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
        # self.nettable.putNumber("State/angle (deg)", self.get_angle().degrees())
        if (c := self.getCurrentCommand()) is not None:
            self.nettable.putString("Running Command", c.getName())
        else:
            self.nettable.putString("Running Command", "None")
        return super().periodic()

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.encoder.get())

    def set_state(self, angle: Rotation2d) -> None:
        self.nettable.putNumber("Commanded/angle (deg)", angle.degrees())
        if self.get_claw_distance() > self.safe_claw_distance:
            self.nettable.putBoolean("Safety/Waiting on Claw", True)
            return
        self.nettable.putBoolean("Safety/Waiting on Claw", False)
        if angle.degrees() < -50:  # more ground pointing
            angle = Rotation2d.fromDegrees(-50)
        elif angle.degrees() > 90:  # more sky pointing
            angle = Rotation2d.fromDegrees(90)
        speed = self.pid.calculate(self.get_angle().radians(), angle.radians())
        speed = 1 if speed > 1 else -1 if speed < -1 else speed
        self.nettable.putNumber("State/Speed (%)", speed)
        self.motor.set(speed)

    def run_angle(self, angle: Rotation2d) -> WrapperCommand:
        return RunCommand(lambda: self.set_state(angle), self).withName(
            f"Set Angle {angle.degrees()} (deg)"
        )

    def angle_intake(self) -> WrapperCommand:
        return self.run_angle(Rotation2d.fromDegrees(55)).withName("Intake")

    def angle_score(self) -> WrapperCommand:
        return self.run_angle(Rotation2d.fromDegrees(-35)).withName("Score")

    def angle_zero(self) -> WrapperCommand:
        return self.run_angle(Rotation2d.fromDegrees(0)).withName("Horizontal")

    def angle_full_up(self) -> WrapperCommand:
        return self.run_angle(Rotation2d.fromDegrees(90)).withName("Max Angle")

    def angle_full_down(self) -> WrapperCommand:
        return self.run_angle(Rotation2d.fromDegrees(90)).withName("Min Angle")
