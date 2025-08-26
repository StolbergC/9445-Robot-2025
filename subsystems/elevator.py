from commands2 import Subsystem

from ntcore import NetworkTableInstance

from wpilib import Mechanism2d, RobotBase, SmartDashboard
from wpilib.simulation import ElevatorSim, RoboRioSim, BatterySim
from wpimath.system.plant import DCMotor
from wpimath.controller import ElevatorFeedforward
from wpimath.units import amperes, meters, kilograms, lbsToKilograms, inchesToMeters

from rev import SparkMax, SparkMaxConfig, SparkBaseConfig, SparkMaxSim


class Elevator(Subsystem):
    kG: float = 0
    kV: float = 0
    kA: float = 0

    kP: float = 100
    kI: float = 0
    kD: float = 0

    current_limit: amperes = 60

    max_height: meters = 1.0

    tolerance: meters = 0.05

    """
    this is meters/rotation
    move the elevator manually to get n rotations, measure height from base of elevator
    this value is (height in meters)/rotations
    """
    conversion_factor: float = 1.0

    nettable_name: str = "000Elevator"

    ### SIMULATION
    """
    This is the number of rotations of the motor to one rotation of the output
    It is used only for simulation
    """
    gearing: float = 3
    moving_mass: kilograms = lbsToKilograms(10)
    drum_radius: meters = inchesToMeters(
        0.25
    )  # radius of the drum that the cable winds on

    def __init__(self) -> None:
        super().__init__()
        self.nettable = NetworkTableInstance.getDefault().getTable(self.nettable_name)
        self.setName(self.nettable_name)

        self.motor_l = SparkMax(24, SparkMax.MotorType.kBrushless)
        self.motor_r = SparkMax(25, SparkMax.MotorType.kBrushless)

        self.setpoint = 0

        self.master_motor_config = SparkMaxConfig()
        self.master_motor_config.closedLoop.P(self.kP).I(self.kI).D(self.kD)
        self.master_motor_config.smartCurrentLimit(self.current_limit)
        self.master_motor_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)

        self.master_motor_config.encoder.positionConversionFactor(
            self.conversion_factor
        )
        self.master_motor_config.encoder.velocityConversionFactor(
            self.conversion_factor / 60
        )

        self.slave_motor_config = (
            SparkMaxConfig()
            .follow(self.motor_l.getDeviceId(), True)
            .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(self.current_limit)
        )

        self.motor_l.configure(
            self.master_motor_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.motor_r.configure(
            self.slave_motor_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.encoder = self.motor_l.getEncoder()

        # self.encoder.setPosition(0)
        self.encoder.setPosition(self.max_height / 2)

        self.closed_loop = self.motor_l.getClosedLoopController()

        self.feedforward = ElevatorFeedforward(0, self.kG, self.kV, self.kA)

        self.mech = Mechanism2d(50, 130 * self.max_height)
        self.mech_root = self.mech.getRoot(self.nettable_name, 25, 0)
        self.mech_lig = self.mech_root.appendLigament("elevator", 10, 90)

        if RobotBase.isSimulation():
            self.ele_sim = ElevatorSim(
                DCMotor.NEO(2),
                self.gearing,
                self.moving_mass,
                self.drum_radius,
                0,
                self.max_height * 1.1,
                True,
                # 0,
                self.get_height(),
            )

            self.motor_l_sim = SparkMaxSim(self.motor_l, DCMotor.NEO(1))
            self.motor_r_sim = SparkMaxSim(self.motor_r, DCMotor.NEO(1))

        SmartDashboard.putData("Elevator Mech", self.mech)
        SmartDashboard.putData(self)

    def periodic(self) -> None:
        self.nettable.putNumber("setpoint (m)", self.setpoint)
        self.nettable.putNumber("current_position (m)", self.get_height())
        self.nettable.putNumber(
            "current_velocity (mps)", velocity := self.encoder.getVelocity()
        )
        self.nettable.putNumber("Closed Loop Error", self.setpoint - self.get_height())

        # this allows for
        ff = self.feedforward.calculate(velocity)
        self.closed_loop.setReference(
            self.setpoint,
            SparkMax.ControlType.kPosition,
            arbFeedforward=ff,
            arbFFUnits=self.closed_loop.ArbFFUnits.kVoltage,
        )

        self.mech_lig.setLength(100 * self.encoder.getPosition() + 10)

        self.nettable.putNumber("ArbFF", ff)
        self.nettable.putNumber("Motor_l Output %", self.motor_l.get())
        self.nettable.putNumber("Motor_r Output %", self.motor_r.get())

    def simulationPeriodic(self) -> None:
        self.ele_sim.setInputVoltage(RoboRioSim.getVInVoltage())
        self.ele_sim.setInput(
            [self.motor_l.getAppliedOutput() * RoboRioSim.getVInVoltage()]
        )

        self.ele_sim.update(0.02)

        vel = self.ele_sim.getVelocity()

        self.motor_l_sim.iterate(
            vel / self.conversion_factor, RoboRioSim.getVInVoltage(), 0.02
        )
        self.motor_r_sim.iterate(
            vel / self.conversion_factor, RoboRioSim.getVInVoltage(), 0.02
        )

        # this number ends up really big and would cause brown out if real
        # it does not soft limit current, so the elevator is too fast in simulation
        self.motor_l_sim.setMotorCurrent(self.ele_sim.getCurrentDraw() / 2)
        self.motor_r_sim.setMotorCurrent(self.ele_sim.getCurrentDraw() / 2)

        RoboRioSim.setVInVoltage(BatterySim.calculate([self.ele_sim.getCurrentDraw()]))

    def get_height(self) -> meters:
        """
        This can be done by simply returning the position.
        Alternatively, take measurements of encoder positions at a number of heights
        and then take a linear (or quadratic or cubic or etc) regression and use it to calculate height

        See this graph with some pre-built regressions
        https://www.desmos.com/calculator/ylq4aebgkp
        """
        return self.encoder.getPosition()
        """Example regression code"""
        """
        a = 0.03
        b = 1.05
        # the / self.conversion factor is not needed if it is 1. 
        # this is just designed to be a drop in replacement method
        # The / self.conversion_factor undoes scaling done by the controller 
        encoder_counts = self.encoder.getPosition() / self.conversion_factor
        return a * (encoder_counts**2) + b * encoder_counts
        """

    def get_setpoint(self) -> meters:
        return self.setpoint

    def set_setpoint(self, setpoint: meters) -> None:
        """
        If a regression is used in get_height, it should also be used here to invert the function
        The implementation is up to the user.
        It should be trivial for linear, quite simple for quadratic, and somewhat more complicated afterwards
        """
        if setpoint < 0:
            self.setpoint = 0
        elif setpoint > self.max_height:
            self.setpoint = self.max_height
        else:
            self.setpoint = setpoint

    def at_setpoint(self) -> bool:
        return abs(self.get_height() - self.setpoint) < self.tolerance

    def stop(self) -> None:
        """
        This method should probably not be used anywhere but is here incase needed elsewhere
        """
        self.closed_loop.setReference(
            self.encoder.getPosition(), SparkMax.ControlType.kPosition
        )
        self.motor_l.stopMotor()
        # maybe not needed. Here for safety
        self.motor_r.stopMotor()

    def set_motors(self, power: float) -> None:
        """
        This should only be used in manual control as apart of a command.
        DO NOT POLL A CONTROLLER AND FEED THIS IN A PERIODIC IN REAL ROBOT CODE
        """
        self.motor_l.set(power)
        # motor_r set by following

    def reset_position(self, position: meters) -> None:
        # self.encoder.setPosition(position / self.conversion_factor)
        # not sure which is right
        self.encoder.setPosition(position)
