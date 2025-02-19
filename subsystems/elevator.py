from typing import Callable
from math import pi

from rev import (
    SparkBaseConfig,
    SparkMax,
    SparkLowLevel,
    SparkMaxConfig,
    SparkBase,
    SparkMaxSim,
)

from wpilib import DigitalInput, RobotBase, RobotController, Mechanism2d, SmartDashboard
from wpilib.simulation import ElevatorSim, RoboRioSim, BatterySim
from wpimath.system.plant import DCMotor


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

heights = {1: 1, 2: 2, 3: 3}


class Elevator(Subsystem):
    def __init__(
        self,
        get_wrist_angle: Callable[[], Rotation2d],
        wrist_length: feet = 1,
    ):
        super().__init__()

        self.has_homed = False

        self.get_wrist_angle = get_wrist_angle
        self.wrist_length = wrist_length

        self.motor = SparkMax(24, SparkLowLevel.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()

        self.motor_config = SparkMaxConfig().smartCurrentLimit(80).inverted(False)
        self.motor_config.encoder.positionConversionFactor(
            pi * 1.12 / 15  # TODO: Find what the conversion factor needs to be
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

        if not RobotBase.isReal():
            self.gearbox = DCMotor.NEO(1)
            self.motor_sim = SparkMaxSim(self.motor, self.gearbox)
            self.elevator_sim = ElevatorSim(
                self.gearbox,
                15,
                6.80,
                inchesToMeters(1.12 / 2),
                inchesToMeters(9),
                inchesToMeters(60),
                True,
                inchesToMeters(9),
            )
        self.mech = Mechanism2d(0.5, 2.5)
        self.root = self.mech.getRoot("elevator", 0.25, 0.25)
        self.mech_base = self.root.appendLigament("ElevatorBase", 0.25, 90)
        self.mech_elevator = self.mech_base.appendLigament(
            "Elevator Immutable", 0.25, 0
        )
        self.mech_elevator_mutable = self.mech_base.appendLigament(
            "Elevator Mutable", 0, 0
        )
        SmartDashboard.putData("ElevatorMech", self.mech)

    def periodic(self) -> None:
        if not self.bottom_limit.get():
            self.encoder.setPosition(0)
            self.has_homed = True

        self.nettable.putNumber("State/position (in)", self.get_position())
        self.nettable.putNumber("At Bottom ?", self.bottom_limit.get())
        self.nettable.putNumber(
            "State/Current Draw (amp)", self.motor.getOutputCurrent()
        )
        return super().periodic()

    def simulationPeriodic(self) -> None:
        self.motor_sim.setBusVoltage(RobotController.getBatteryVoltage())
        self.elevator_sim.setInput(
            [self.motor.getAppliedOutput() * RoboRioSim.getVInVoltage()]
        )
        self.elevator_sim.update(0.02)
        self.nettable.putNumber(
            "Sim/Position (in)", self.elevator_sim.getPositionInches()
        )
        self.nettable.putNumber("Sim/output (%)", self.elevator_sim.getOutput()[0])
        self.nettable.putNumber(
            "Sim/Velocity (fps)", self.elevator_sim.getVelocityFps()
        )
        # self.encoder.setPosition(self.elevator_sim.getPositionInches())
        self.motor_sim.iterate(
            self.elevator_sim.getVelocity(), RoboRioSim.getVInVoltage(), 0.2
        )
        RoboRioSim.setVInVoltage(
            BatterySim.calculate([self.elevator_sim.getCurrentDraw()])
        )
        self.mech_elevator_mutable.setLength(self.elevator_sim.getPosition())
        return super().simulationPeriodic()

    def get_position(self) -> feet:
        return self.encoder.getPosition()

    def set_state(self, position: feet) -> None:
        # This assumes that zero degrees is in the center, and that it decreases as the wrist looks closer to the ground
        if abs(self.get_wrist_angle().degrees()) > 10:
            self.nettable.putBoolean("Safety/Waiting on Wrist", True)
            return
        self.nettable.putBoolean("Safety/Waiting on Wrist", False)
        self.nettable.putNumber("Commanded/position (in)", position)
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
        def set_homed():
            self.has_homed = True

        return (
            RunCommand(lambda: self.motor.set(-0.25), self)
            .onlyWhile(
                lambda: self.bottom_limit.get()  # maybe will be not limit.get() based on wiring
            )
            .andThen(
                RunCommand(lambda: self.motor.set(0.1)).until(
                    lambda: self.motor.getOutputCurrent() > 10
                )
            )
            .andThen(InstantCommand(lambda: self.motor.set(0), self))
            .andThen(InstantCommand(set_homed))
        )

    def command_position(self, position: feet) -> WrapperCommand:
        if self.has_homed:
            return RunCommand(lambda: self.set_state(position), self).withName(
                f"Set Position to {position} ft"
            )
        else:
            return (
                self.home()
                .andThen(RunCommand(lambda: self.set_state(position)))
                .onlyWhile(lambda: abs(self.get_position() - position) > 2)
                .withName("Home then Set Position to {position} ft")
            )

    def follow_setpoint(self, level: Callable[[], int]) -> RunCommand:
        return RunCommand(lambda: self.set_state(level()), self)

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

    def reset(self) -> InstantCommand:
        return InstantCommand(lambda: self.encoder.setPosition(0))
