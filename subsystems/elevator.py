from typing import Callable
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
from wpimath.geometry import Rotation2d

from commands2 import (
    InstantCommand,
    RunCommand,
    SequentialCommandGroup,
    Subsystem,
    WrapperCommand,
)


class Elevator(Subsystem):
    def __init__(
        self,
        get_wrist_angle: Callable[[], Rotation2d],
        safe_wrist_angle: Rotation2d,
        wrist_length: feet = 1,
    ):
        super().__init__()

        self.has_homed = False

        self.get_wrist_angle = get_wrist_angle
        self.safe_after_wrist_angle = safe_wrist_angle
        self.wrist_length = wrist_length

        self.motor = SparkMax(24, SparkLowLevel.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()

        self.motor_config = SparkMaxConfig().smartCurrentLimit(40).inverted(False)
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
        self.bottom_height: feet = 9 / 12
        # TODO: Maybe? The 6 inches are the overlap desired
        self.top_height: feet = (
            self.bottom_height * 12 + 2 * inches(29) - 2 * inches(6)
        ) / 12

    def periodic(self) -> None:
        if self.bottom_limit.get():
            self.encoder.setPosition(0)

        self.nettable.putNumber("State/position (ft)", self.get_position())
        self.nettable.putNumber("At Bottom ?", self.bottom_limit.get())
        return super().periodic()

    def get_position(self) -> feet:
        return self.encoder.getPosition()

    def set_state(self, position: feet) -> None:
        # This assumes that zero degrees is in the center, and that it decreases as the wrist looks closer to the ground
        if self.get_wrist_angle().radians() > self.safe_after_wrist_angle.radians():
            self.nettable.putBoolean("Safety/Waiting on Wrist", True)
            return
        self.nettable.putBoolean("Safety/Waiting on Wrist", False)
        position = self._make_position_safe(position)
        if position < self.bottom_height:
            position = self.bottom_height
        elif position > self.top_height:
            position = self.top_height
        speed = self.pid.calculate(self.get_position(), position)
        speed = 1 if speed > 1 else -1 if speed < -1 else speed
        self.motor.set(speed)

    def _make_position_safe(self, position: feet) -> feet:
        """
        This assumes that the wrist needs to point at the angle it is currently at
        and makes it so that the elevator will not go boom
        """
        # - sin b/c + and - 90_deg are swapped
        pointed_at = self.get_wrist_angle().sin() * self.wrist_length + position
        if self.bottom_height < pointed_at and pointed_at < self.top_height:
            self.nettable.putBoolean("Safety/Adjusting Position", False)
            return position
        if pointed_at > self.top_height:
            self.nettable.putBoolean("Safety/Adjusting Position", True)
            return self.top_height - self.get_wrist_angle().sin() * self.wrist_length
        self.nettable.putBoolean("Safety/Adjusting Position", True)
        return self.bottom_height - self.get_wrist_angle().sin() * self.wrist_length

    def home(self) -> SequentialCommandGroup:
        return (
            RunCommand(lambda: self.motor.set(-0.25), self)
            .onlyWhile(
                lambda: self.bottom_limit.get()  # maybe will be not limit.get() based on wiring
            )
            .andThen(InstantCommand(lambda: self.motor.set(0), self))
        )

    def command_position(self, position: feet) -> WrapperCommand:
        if self.has_homed:
            return RunCommand(lambda: self.set_state(position), self).withName(
                f"Set Position to {position} ft"
            )
        else:
            self.has_homed = True
            return (
                self.home()
                .andThen(RunCommand(lambda: self.set_state(position)))
                .withName("Home then Set Position to {position} ft")
            )

    # TODO: None of these heights are correct. They depend on the angle and stuff
    def command_l1(self) -> WrapperCommand:
        return self.command_position(1).withName("L1")

    def command_l2(self) -> WrapperCommand:
        return self.command_position(1).withName("L2")

    def command_l3(self) -> WrapperCommand:
        return self.command_position(1).withName("L3")

    def manual_control(self, power: float) -> None:
        power = 0.5 if power > 0.5 else -0.5 if power < -0.5 else power
        self.motor.set(power)
