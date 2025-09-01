from math import pi

from wpilib import Mechanism2d, RobotBase, SmartDashboard, Timer
from wpilib.simulation import ElevatorSim, RoboRioSim

from wpimath.units import (
    meters,
    inchesToMeters,
    kilograms,
    metersToInches,
    meters_per_second,
    amperes,
)
from wpimath.system.plant import DCMotor

from commands2 import Subsystem

from ntcore import NetworkTableInstance


from rev import SparkMax, SparkBaseConfig, SparkMaxSim


class Claw(Subsystem):
    kP: float = 0.26
    kI: float = 0
    kD: float = 0.14

    TEETH = 18
    DIAMETERAL_PITCH = 20
    PCD = TEETH / DIAMETERAL_PITCH

    gearing = 5

    max_extention: meters = inchesToMeters(16)

    current_limit: amperes = 35

    # this should be configured such that positive power moves the fingers apart
    inverted: bool = False

    tolerance: meters = inchesToMeters(1)

    # SIMULATION
    moving_mass: kilograms = 0.5

    def __init__(self):
        self.nettable = NetworkTableInstance.getDefault().getTable("000Claw")
        self.setName("000Claw")

        self.motor = SparkMax(28, SparkMax.MotorType.kBrushless)

        self.encoder = self.motor.getEncoder()

        motor_config = SparkBaseConfig()
        motor_config.setIdleMode(
            SparkBaseConfig.IdleMode.kCoast,
        ).smartCurrentLimit(
            self.current_limit,
        ).inverted(
            self.inverted,
        )
        motor_config.closedLoop.pid(
            self.kP,
            self.kI,
            self.kD,
        )
        motor_config.encoder.positionConversionFactor(
            pi * self.PCD / self.gearing
        ).velocityConversionFactor(pi * self.PCD / (self.gearing * 60))

        self.motor.configure(
            motor_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.closed_loop = self.motor.getClosedLoopController()

        self.setpoint: meters = self.get_distance()

        self.stall_timer = Timer()

        self.mech = Mechanism2d(self.max_extention * 120, self.max_extention * 60)
        self.mech_root = self.mech.getRoot(
            "Claw", self.max_extention * 60, self.max_extention * 30
        )
        self.mech_lig_left = self.mech_root.appendLigament(
            "ClawLeft", inchesToMeters(1), 0
        )
        self.mech_lig_right = self.mech_root.appendLigament(
            "ClawRight", inchesToMeters(1), 180
        )
        self.mech_finger_left_lig = self.mech_lig_left.appendLigament(
            "FingerL", self.max_extention * 15, 90
        )

        self.mech_finger_right_lig = self.mech_lig_right.appendLigament(
            "FingerR", self.max_extention * 15, -90
        )

        if RobotBase.isSimulation():
            gearbox = DCMotor.NEO()
            self.motor_sim = SparkMaxSim(self.motor, gearbox)
            self.encoder_sim = self.motor_sim.getRelativeEncoderSim()
            # a rack and pinion is basically an elevator on its side
            # so that is the sim method we use
            self.sim = ElevatorSim(
                gearbox,
                self.gearing,
                self.moving_mass,
                self.PCD,
                1.5,
                metersToInches(self.max_extention),
                False,
                2,
            )

        SmartDashboard.putData(self)
        SmartDashboard.putData("ClawMech", self.mech)

    def periodic(self) -> None:
        dist = self.get_distance()
        self.nettable.putNumber("Distance/inches", metersToInches(dist))
        self.nettable.putNumber("Distance/meters", dist)

        self.nettable.putNumber("Setpoint/inches", metersToInches(self.setpoint))
        self.nettable.putNumber("Setpoint/meters", self.setpoint)

        self.nettable.putNumber("Error/inches", metersToInches(self.setpoint - dist))
        self.nettable.putNumber("Error/meters", self.setpoint - dist)

        velocity = self.get_velocity()
        self.nettable.putNumber("Velocity/inches per second", metersToInches(velocity))
        self.nettable.putNumber("Velocity/meters per second", velocity)

        self.mech_lig_left.setLength(dist / 2 * 100)
        self.mech_lig_right.setLength(dist / 2 * 100)

        self.closed_loop.setReference(
            metersToInches(self.setpoint), SparkMax.ControlType.kPosition
        )

        self.nettable.putNumber("Output %", self.motor.getAppliedOutput())

        current = self.motor.getOutputCurrent()
        self.nettable.putNumber("Current", current)

        if (
            abs(current) >= self.current_limit * 0.9
            and abs(velocity) <= 0.05
            and not self.stall_timer.isRunning()
        ):
            self.stall_timer.start()
        if (
            abs(current) < self.current_limit * 0.9 or abs(velocity) > 0.05
        ) and self.stall_timer.isRunning():
            self.stall_timer.stop()
            self.stall_timer.reset()

        if self.stall_timer.hasElapsed(0.5):
            # outside
            if self.motor.get() > 0:
                self.encoder.setPosition(metersToInches(self.max_extention))
            else:
                self.encoder.setPosition(0)

        self.nettable.putBoolean("Stall/IsStalling", self.stall_timer.isRunning())
        self.nettable.putNumber("Stall/Time (s)", self.stall_timer.get())

    def simulationPeriodic(self) -> None:
        self.sim.update(0.02)

        self.motor_sim.iterate(self.sim.getVelocity(), RoboRioSim.getVInVoltage(), 0.02)

        # self.encoder_sim.setPosition()

        RoboRioSim.setVInCurrent(self.motor.getOutputCurrent())

        self.sim.setInputVoltage(
            self.motor.getAppliedOutput() * RoboRioSim.getVInVoltage()
        )

    def get_distance(self) -> meters:
        return inchesToMeters(self.encoder.getPosition())

    def get_velocity(self) -> meters_per_second:
        return inchesToMeters(self.encoder.getVelocity())

    def get_setpoint(self) -> meters:
        return self.setpoint

    def set_setpoint(self, setpoint: meters) -> None:
        if setpoint < 0:
            setpoint = 0
        if setpoint > self.max_extention:
            setpoint = self.max_extention
        self.setpoint = setpoint

    def at_setpoint(self) -> bool:
        return abs(self.get_distance() - self.setpoint) < self.tolerance

    def at_center(self) -> bool:
        return self.stall_timer.isRunning() and self.motor.get() < 0

    def at_outside(self) -> bool:
        return self.stall_timer.isRunning() and self.motor.get() > 0
