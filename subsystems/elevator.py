from commands2 import Subsystem

from ntcore import NetworkTableInstance

from wpimath.system.plant import DCMotor
from wpimath.controller import ElevatorFeedforward
from wpimath.units import amperes

from rev import SparkMax, SparkMaxConfig, SparkBaseConfig


class Elevator(Subsystem):
    kG: float = 0
    kV: float = 0
    kA: float = 0

    kP: float = 0
    kI: float = 0
    kD: float = 0

    current_limit: amperes = 60

    """
    this is meters/rotation
    move the elevator manually to get n rotations, measure height from base of elevator
    this value is (height in meters)/rotations
    """
    conversion_factor: float = 1.0

    nettable_name: str = "000Elevator"

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

        self.encoder.setPosition(0)

        self.closed_loop = self.motor_l.getClosedLoopController()

        self.feedforward = ElevatorFeedforward(0, self.kG, self.kV, self.kA)

    def periodic(self) -> None:
        self.nettable.putNumber("setpoint (m)", self.setpoint)
        self.nettable.putNumber("current_position (m)", self.encoder.getPosition())
        self.nettable.putNumber(
            "current_velocity (mps)", velocity := self.encoder.getVelocity()
        )

        ff = self.feedforward.calculate(velocity)
        self.closed_loop.setReference(
            self.setpoint,
            SparkMax.ControlType.kPosition,
            arbFeedforward=ff,
            arbFFUnits=self.closed_loop.ArbFFUnits.kVoltage,
        )

        self.nettable.putNumber("ArbFF", ff)
        self.nettable.putNumber("Motor_l Output %", self.motor_l.get())
        self.nettable.putNumber("Motor_r Output %", self.motor_r.get())

        return super().periodic()
