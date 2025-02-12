from math import pi
from typing import Callable

from ntcore import Event, EventFlags, NetworkTable, NetworkTableInstance, ValueEventData
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d
from wpimath.units import feet

from wpilib import DigitalInput

from commands2 import InstantCommand, RunCommand, Subsystem, WrapperCommand
from rev import SparkMax, SparkLowLevel, SparkMaxConfig


class Claw(Subsystem):
    def __init__(self) -> None:
        TEETH = 18
        DIAMETERAL_PITCH = 20  # units??? maybe /= 12
        PCD = TEETH / DIAMETERAL_PITCH

        super().__init__()

        self.nettable = NetworkTableInstance.getDefault().getTable("000Claw")

        self.motor = SparkMax(28, SparkLowLevel.MotorType.kBrushless)
        motor_config = SparkMaxConfig()
        motor_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake).smartCurrentLimit(
            20
        ).encoder.positionConversionFactor(
            pi * PCD / (9 * 4096)  # TODO: Insert Gear Ratio Here
        )

        self.encoder = self.motor.getEncoder()
        self.encoder.setPosition(0)
        # max of 1 ft/s and accelerate in 10s
        self.pid = ProfiledPIDController(
            0.075, 0, 0, TrapezoidProfile.Constraints(1, 10)
        )

        self.inside_limit = DigitalInput(2)
        self.outside_limit = DigitalInput(3)

        def nettable_listener(_nt: NetworkTable, key: str, ev: Event):
            if isinstance(v := ev.data, ValueEventData):
                if key == "PID/p":
                    self.pid.setP(v.value.value())
                elif key == "PID/i":
                    self.pid.setI(v.value.value())
                elif key == "PID/d":
                    self.pid.setD(v.value.value())
                elif key == "Config/Velocity (ft/s)":
                    self.pid.setConstraints(
                        TrapezoidProfile.Constraints(
                            v.value.value(), v.value.value() * 10
                        )
                    )

        self.nettable.addListener(EventFlags.kValueAll, nettable_listener)

        self.nettable.putNumber("PID/p", self.pid.getP())
        self.nettable.putNumber("PID/i", self.pid.getI())
        self.nettable.putNumber("PID/d", self.pid.getD())

        self.nettable.putNumber(
            "Config/Velocity (ft/s)", self.pid.getConstraints().maxVelocity
        )

    def periodic(self) -> None:
        if not self.inside_limit.get():
            self.encoder.setPosition(0)
        if not self.outside_limit.get():
            self.encoder.setPosition(
                2  # maybe will have to multiply by the position conversion factor
            )  # this is the max distance between the fingers in feet
        self.nettable.putBoolean(
            "State/inside limit switch", not self.inside_limit.get()
        )
        self.nettable.putBoolean(
            "State/outside limit switch", not self.outside_limit.get()
        )
        self.nettable.putNumber("State/Distance (in)", self.get_dist())
        if (c := self.getCurrentCommand()) is not None:
            self.nettable.putString("Running Command", c.getName())
        else:
            self.nettable.putString("Running Command", "None")

        return super().periodic()

    def get_dist(self) -> float:
        """the distance in inches"""
        return self.encoder.getPosition()

    def set_motor(self, power: float) -> None:
        power = 1 if power > 1 else -1 if power < -1 else power
        if (
            power > 0
            and not self.outside_limit.get()
            or power < 0
            and not self.inside_limit.get()
        ):
            self.nettable.putNumber("State/Out Speed (%)", 0)
            return
        self.nettable.putNumber("State/Out Speed (%)", power)
        self.motor.set(power)

    def set_motor_lambda(self, power: Callable[[], float]):
        self.set_motor(power())

    def set_position(self, distance: float) -> WrapperCommand:
        """the distance is in inches"""
        return (
            RunCommand(
                lambda: self.set_motor(self.pid.calculate(self.get_dist(), distance))
            )
            .onlyWhile(lambda: abs(self.get_dist() - distance) > 0.5)
            .withName(f"Go to {distance} ft")
        )

    def algae(self) -> WrapperCommand:
        return self.set_position(16).withName("Grab Algae")

    def coral(self) -> WrapperCommand:
        return self.set_position(0).withName("Grab Coral")

    def cage(self) -> WrapperCommand:
        return self.set_position(2).withName("Inside of Cage")

    def reset_position(self) -> None:
        self.encoder.setPosition(0)

    def reset(self) -> InstantCommand:
        return InstantCommand(self.reset_position)
