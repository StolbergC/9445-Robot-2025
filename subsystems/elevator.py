from typing import Callable
from math import floor, pi

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
    feetToMeters,
    feet,
    inches,
)
from wpimath.controller import ProfiledPIDController, ElevatorFeedforward, PIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d

from commands2 import (
    DeferredCommand,
    InstantCommand,
    ParallelCommandGroup,
    ParallelRaceGroup,
    RepeatCommand,
    RunCommand,
    SequentialCommandGroup,
    Subsystem,
    WaitCommand,
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

        self.spool_diameter = 0.95  # inches
        self.spool_depth = 0.61  # inches
        self.rope_diameter = 0.12  # inches

        self.rope_area_constant = (pi * ((r := self.rope_diameter / 2) * r)) / (
            self.rope_diameter * self.rope_diameter
        )

        if RobotBase.isReal():
            self.has_homed = False
        else:
            self.has_homed = True

        self.get_wrist_angle = get_wrist_angle
        self.wrist_length = wrist_length

        self.motor = SparkMax(24, SparkLowLevel.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()

        self.motor_config = (
            SparkMaxConfig()
            .smartCurrentLimit(50)
            .inverted(False)
            .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        )
        self.motor_config.encoder.positionConversionFactor(
            1 / 15  # TODO: Find what the conversion factor needs to be
        ).velocityConversionFactor(1 / 15)
        self.encoder.setPosition(0)

        self.motor.configure(
            self.motor_config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        # TODO: Check if this is actually the bottom, what it means, how we use it, etc.
        self.bottom_limit = DigitalInput(0)

        self.pid = PIDController(30, 0, 0)
        # self.pid = ProfiledPIDController(
        #     13, 0, 0, TrapezoidProfile.Constraints(v := feetToMeters(5), v * 4)
        # )
        self.feedforward = ElevatorFeedforward(0, 0.55, 0, 0)

        self.nettable = NetworkTableInstance.getDefault().getTable("000Elevator")

        def nettable_updater(_nt: NetworkTable, key: str, ev: Event) -> None:
            if isinstance(data := ev.data, ValueEventData):
                if key == "PID/p":
                    self.pid.setP(data.value.value())
                elif key == "PID/i":
                    self.pid.setI(data.value.value())
                elif key == "PID/d":
                    self.pid.setD(data.value.value())
                elif key == "Feedforward/kS":
                    self.feedforward = ElevatorFeedforward(
                        data.value.value(),
                        self.feedforward.getKg(),
                        self.feedforward.getKv(),
                        self.feedforward.getKa(),
                    )
                elif key == "Feedforward/kG":
                    self.feedforward = ElevatorFeedforward(
                        self.feedforward.getKs(),
                        data.value.value(),
                        self.feedforward.getKv(),
                        self.feedforward.getKa(),
                    )
                elif key == "Feedforward/kV":
                    self.feedforward = ElevatorFeedforward(
                        self.feedforward.getKs(),
                        self.feedforward.getKg(),
                        data.value.value(),
                        self.feedforward.getKa(),
                    )
                elif key == "Feedforward/kA":
                    self.feedforward = ElevatorFeedforward(
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

        self.bottom_height: float = 0
        self.top_height: float = 20

        if not RobotBase.isReal():
            self.gearbox = DCMotor.NEO(1)
            self.motor_sim = SparkMaxSim(self.motor, self.gearbox)
            self.elevator_sim = ElevatorSim(
                self.gearbox,
                15,
                6.80,
                inchesToMeters(self.spool_diameter / 2),
                inchesToMeters(self.bottom_height),
                inchesToMeters(self.top_height),
                True,
                inchesToMeters(self.bottom_height),
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

        self.setpoint = 0

    def periodic(self) -> None:
        self.nettable.putNumber("Commanded/position", self.setpoint)
        self.nettable.putBoolean("Commanded/close", self.close())
        self.nettable.putNumber("State/position (in)", self.get_position())
        self.nettable.putNumber(
            "State/raw_position (rotations)", self.encoder.getPosition()
        )
        self.nettable.putNumber("State/velocity (in per s)", self.get_velocity())
        self.nettable.putNumber("At Bottom ?", self.bottom_limit.get())
        self.nettable.putNumber(
            "State/Current Draw (amp)", self.motor.getOutputCurrent()
        )
        if (c := self.getCurrentCommand()) is not None:
            self.nettable.putString("Running Command", c.getName())
        else:
            self.nettable.putString("Running Command", "None")
        return super().periodic()

    def stop(self) -> InstantCommand:
        return InstantCommand(lambda: self.motor.set(0))

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
        self.motor_sim.iterate(
            self.elevator_sim.getVelocity(), RoboRioSim.getVInVoltage(), 0.2
        )
        RoboRioSim.setVInVoltage(
            BatterySim.calculate([self.elevator_sim.getCurrentDraw()])
        )
        self.mech_elevator_mutable.setLength(self.elevator_sim.getPosition())
        return super().simulationPeriodic()

    def get_position(self) -> feet:
        # this is based loosely on integration with washers for volume
        outer_radius = (
            self.spool_diameter
            + 2
            * self.rope_diameter
            * 0.3192
            * (-self.encoder.getPosition() * (self.spool_depth / self.rope_diameter))
        ) / 2
        return self.bottom_height + (
            (
                (outer_radius * outer_radius)
                - ((spool_radius := self.spool_diameter / 2) * spool_radius)
            )
            * self.spool_depth
            * pi
        ) / (pi * (spool_radius * spool_radius))

    def get_velocity(self) -> float:
        # this is based loosely on integration with washers for volume
        outer_radius = (
            self.spool_diameter
            + 2
            * self.rope_diameter
            * 0.3192
            * (self.encoder.getVelocity() * (self.spool_depth / self.rope_diameter))
        ) / 2
        return (
            (
                (outer_radius * outer_radius)
                - ((spool_radius := self.spool_diameter / 2) * spool_radius)
            )
            * self.spool_depth
            * pi
        ) / (pi * (spool_radius * spool_radius))

    def set_state(self, position: feet) -> None:
        # This assumes that zero degrees is in the center, and that it decreases as the wrist looks closer to the ground
        if abs(self.get_wrist_angle().degrees() - 10) > 10:
            self.nettable.putBoolean("Safety/Waiting on Wrist", True)
            self.motor.set(0)
            return
        self.nettable.putBoolean("Safety/Waiting on Wrist", False)
        self.nettable.putNumber("Commanded/position (in)", position)
        if position < self.bottom_height:
            position = self.bottom_height
        elif position > self.top_height:
            position = self.top_height
        volts = self.pid.calculate(
            self.encoder.getPosition(), position
        ) + self.feedforward.calculate(0, 0)
        self.nettable.putNumber("State/Out Power (V)", volts)
        volts = -9 if volts < -9 else volts
        self.motor.setVoltage(volts)

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

    def tighten(self) -> WrapperCommand:
        return (
            RunCommand(lambda: self.motor.set(0.25))
            .until(lambda: self.motor.getOutputCurrent() > 3)
            .andThen(InstantCommand(lambda: self.motor.set(0)))
            .withName("Tighten Rope")
        )

    def command_position(self, position: float) -> WrapperCommand:
        return (
            # self.set_setpoint(position)
            # .andThen(
            RunCommand(lambda: self.set_state(position), self)
            # )
            .until(lambda: abs(self.encoder.getPosition() - position) < 0.25)
            .andThen(self.stop())
            .withName(f"Set Position to {position} ft")
        )

    def follow_setpoint(self) -> RepeatCommand:
        return RepeatCommand(
            DeferredCommand(lambda: InstantCommand(self.set_state(self.setpoint)), self)
        )

    def command_bottom(self) -> WrapperCommand:
        return self.command_position(0).withName("Bottom")

    def command_l1(self) -> WrapperCommand:
        return self.command_position(5.827).withName("L1")

    def command_l2(self) -> WrapperCommand:
        return self.command_position(15.15).withName("L2")

    def command_l3(self) -> WrapperCommand:
        return self.command_position(self.top_height - 0.25).withName("L3")

    def command_intake(self) -> WrapperCommand:
        return self.command_position(4).withName("Intake")  # 21.95 in

    def algae_intake_low(self) -> WrapperCommand:
        return self.command_position(10).withName("Algae Low")

    def algae_intake_high(self) -> WrapperCommand:
        return self.command_position(self.top_height - 0.5).withName("Algae high")

    def command_processor(self) -> WrapperCommand:
        return self.command_position(1.5).withName("Processor")  # this is a guess

    def set_setpoint(self, setpoint: float) -> InstantCommand:
        def do_it():
            self.setpoint = setpoint

        return InstantCommand(do_it)

    def set_setpoint_bottom(self) -> WrapperCommand:
        return self.set_setpoint(0).withName("Bottom")

    def set_setpoint_l1(self) -> WrapperCommand:
        return self.set_setpoint(5.827).withName("L1")

    def set_setpoint_l2(self) -> WrapperCommand:
        return self.set_setpoint(15.15).withName("L2")

    def set_setpoint_l3(self) -> WrapperCommand:
        return self.set_setpoint(self.top_height).withName("L3")

    def set_setpoint_intake(self) -> WrapperCommand:
        return self.set_setpoint(3.5).withName("Intake")  # 21.95 in

    def set_setpoint_algae_intake_low(self) -> WrapperCommand:
        return self.set_setpoint(10).withName("Algae Low")

    def set_setpoint_algae_intake_high(self) -> WrapperCommand:
        return self.set_setpoint(self.top_height - 0.5).withName("Algae high")

    def set_setpoint_processor(self) -> WrapperCommand:
        return self.set_setpoint(1.5).withName("Processor")  # this is a guess

    def close(self) -> bool:
        return abs(self.setpoint - self.encoder.getPosition()) < 0.25

    def wait_until_close(self) -> ParallelRaceGroup:
        return RepeatCommand(WaitCommand(0.1)).until(self.close)

    def manual_control(self, power: float) -> None:
        power = 0.5 if power > 0.5 else -0.5 if power < -0.5 else power
        self.motor.set(-power)

    def reset(self) -> InstantCommand:
        def go() -> None:
            self.encoder.setPosition(0)

        return InstantCommand(go)
