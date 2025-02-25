from math import pi
from typing import Callable
from time import time

import commands2
from ntcore import Event, EventFlags, NetworkTable, NetworkTableInstance, ValueEventData
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d
from wpimath.units import feet

from wpilib import DigitalInput

from commands2 import (
    DeferredCommand,
    InstantCommand,
    RunCommand,
    Subsystem,
    WrapperCommand,
)
from rev import SparkMax, SparkLowLevel, SparkMaxConfig, SparkBase


class Claw(Subsystem):
    def __init__(
        self, get_wrist_angle: Callable[[], Rotation2d], safe_to_move_after: Rotation2d
    ) -> None:
        TEETH = 18
        DIAMETERAL_PITCH = 20  # units??? maybe /= 12
        PCD = TEETH / DIAMETERAL_PITCH

        super().__init__()
        self.get_wrist_angle = get_wrist_angle
        self.safe_to_move = safe_to_move_after

        self.nettable = NetworkTableInstance.getDefault().getTable("000Claw")

        self.motor = SparkMax(28, SparkLowLevel.MotorType.kBrushless)
        motor_config = SparkMaxConfig()
        motor_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake).smartCurrentLimit(
            20
        ).encoder.positionConversionFactor(pi * PCD / 9).velocityConversionFactor(
            pi * PCD / (9 * 60)
        )
        self.motor.configure(
            motor_config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters,
        )

        self.encoder = self.motor.getEncoder()
        self.encoder.setPosition(0)
        # max of 1 ft/s and accelerate in 10s
        self.pid = ProfiledPIDController(
            0.075, 0, 0, TrapezoidProfile.Constraints(12, 120)
        )

        self.stall_timer = time()
        self.is_stalling = True

        self.has_homed = False

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

    def at_center(self) -> bool:
        return (
            self.is_stalling
            and time() - self.stall_timer > 0.125
            and abs(self.encoder.getVelocity()) < 0.25
            and self.motor.getAppliedOutput() > 0
        )

    def at_outside(self) -> bool:
        return (
            self.is_stalling
            and time() - self.stall_timer > 0.125
            and abs(self.encoder.getVelocity()) < 0.25
            and self.motor.getAppliedOutput() < 0
        )

    def periodic(self) -> None:
        if not self.is_stalling and self.motor.getOutputCurrent() > 20:
            self.is_stalling = True
            self.stall_timer = time()

        if self.is_stalling and self.motor.getOutputCurrent() < 15:
            self.is_stalling = False

        if self.at_center():
            self.encoder.setPosition(0)
            self.has_homed = True
        if self.at_outside():
            self.encoder.setPosition(-8.75)
            self.has_homed = True
        self.nettable.putBoolean("State/inside hard stop", self.at_center())
        self.nettable.putBoolean("State/outside hard stop", self.at_outside())
        self.nettable.putNumber("State/Distance (in)", self.get_dist())
        self.nettable.putNumber(
            "State/Current draw (amps)", self.motor.getOutputCurrent()
        )
        if (c := self.getCurrentCommand()) is not None:
            self.nettable.putString("Running Command", c.getName())
        else:
            self.nettable.putString("Running Command", "None")

        return super().periodic()

    def get_dist(self) -> float:
        """the distance in inches"""
        return -2 * self.encoder.getPosition()

    def set_motor(self, power: float) -> float:
        power = (
            0.4 if power > 0.4 else -0.4 if power < -0.4 else power
        )  # TODO: Push this if possible b/c gears are now aluminum
        if (
            (power > 0 and self.at_outside())
            or (power < 0 and self.at_center())
            or self.get_wrist_angle().radians() > self.safe_to_move.radians()
        ):
            if self.get_wrist_angle().radians() > self.safe_to_move.radians():
                self.nettable.putBoolean("Safety/Waiting on Wrist", True)
            else:
                self.nettable.putBoolean("Safety/Waiting on Wrist", False)
            self.nettable.putNumber("State/Out Speed (%)", 0)
            return 0
        self.nettable.putBoolean("Safety/Waiting on Wrist", False)
        self.nettable.putNumber("State/Out Speed (%)", power)
        self.motor.set(power)
        return power

    def set_motor_lambda(self, power: Callable[[], float]):
        self.set_motor(power())

    def set_position(self, distance: float) -> WrapperCommand:
        """the distance is in inches"""
        return (
            DeferredCommand(
                lambda: (
                    self.home_outside() if not self.has_homed else commands2.cmd.none()
                )
            )
            .andThen(
                RunCommand(
                    lambda: self.set_motor(
                        -self.pid.calculate(self.get_dist(), distance)
                    )
                )
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

    def home_outside(self) -> WrapperCommand:
        return (
            RunCommand(lambda: self.set_motor(0.25))
            .until(lambda: self.has_homed)
            .withName("Homing Outside")
        )

    def home_inside(self) -> WrapperCommand:
        return (
            RunCommand(lambda: self.set_motor(-0.25))
            .until(lambda: self.has_homed)
            .withName("Homing Outside")
        )
