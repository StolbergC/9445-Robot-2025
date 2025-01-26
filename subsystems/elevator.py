from rev import SparkBaseConfig, SparkMax, SparkLowLevel, SparkMaxConfig, SparkBase

from wpilib import DigitalInput

from ntcore import NetworkTableInstance, EventFlags, Event, ValueEventData, NetworkTable

from wpimath.units import (
    inchesToMeters,
    meters_per_second,
    meters_per_second_squared,
    feetToMeters,
    feet,
    inches,
)

from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile

from commands2 import RunCommand, Subsystem, WrapperCommand


class Elevator(Subsystem):
    def __init__(self):
        super().__init__()

        self.motor = SparkMax(24, SparkLowLevel.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()

        self.motor_config = SparkMaxConfig().smartCurrentLimit(30).inverted(False)
        self.motor_config.encoder.positionConversionFactor(
            1  # TODO: Find what the conversion factor needs to be
        )

        # TODO: check that this is the correct disabling and then set it for the other neo motors
        self.motor_config.signals.absoluteEncoderPositionAlwaysOn(
            False
        ).absoluteEncoderVelocityAlwaysOn(False).analogPositionAlwaysOn(
            False
        ).IAccumulationAlwaysOn(
            False
        ).analogVoltageAlwaysOn(
            False
        ).externalOrAltEncoderPositionAlwaysOn(
            False
        ).absoluteEncoderPositionAlwaysOn(
            False
        ).externalOrAltEncoderPositionAlwaysOn(
            False
        ).externalOrAltEncoderVelocityAlwaysOn(
            False
        )

        self.motor.configure(
            self.motor_config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        # TODO: Check if this is actually the bottom, what it means, how we use it, etc.
        self.bottom_limit = DigitalInput(0)

        self.pid = ProfiledPIDController(
            0.01, 0, 0, TrapezoidProfile.Constraints(v := feetToMeters(5), v * 4)
        )

        self.nettable = NetworkTableInstance.getDefault().getTable("000Elevator")

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

        # TODO: Maybe?
        self.bottom_height: inches = 9
        # TODO: Maybe?
        self.top_height: feet = self.bottom_height + inches(29)

    def periodic(self) -> None:
        if self.bottom_limit.get():
            self.encoder.setPosition(0)

        self.nettable.putNumber("State/position (ft)", self.get_position())
        self.nettable.putNumber("At Bottom ?", self.bottom_limit.get())
        return super().periodic()

    def get_position(self) -> feet:
        return self.encoder.getPosition()

    def set_state(self, position: feet) -> None:
        position_in = position * 12
        if position_in < self.bottom_height:
            position = self.bottom_height / 12
        elif position_in > self.top_height:
            position = self.bottom_height / 12
        speed = self.pid.calculate(self.get_position(), position)
        speed = 1 if speed > 1 else -1 if speed < -1 else speed
        self.motor.set(speed)

    def command_position(self, position: feet) -> WrapperCommand:
        return RunCommand(lambda: self.set_state(position), self).withName(
            f"Set Position to {position} ft"
        )

    # TODO: None of these heights are correct. They depend on the angle and stuff
    def command_l1(self) -> WrapperCommand:
        return self.command_position(1).withName("L1")

    def command_l2(self) -> WrapperCommand:
        return self.command_position(1).withName("L2")

    def command_l3(self) -> WrapperCommand:
        return self.command_position(1).withName("L3")
