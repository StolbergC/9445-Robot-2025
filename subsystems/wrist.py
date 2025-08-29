from commands2 import Subsystem

from ntcore import NetworkTableInstance

from wpilib import Mechanism2d, RobotBase, SmartDashboard
from wpilib.simulation import SingleJointedArmSim, RoboRioSim

from wpimath.units import (
    amperes,
    degrees,
    degrees_per_second,
    degrees_per_second_squared,
    meters,
    kilograms,
)
from wpimath.geometry import Rotation2d
from wpimath.system.plant import DCMotor
from wpimath.controller import ArmFeedforward

from rev import (
    MAXMotionConfig,
    SparkBase,
    SparkMax,
    SparkBaseConfig,
    SparkMaxSim,
    SparkAbsoluteEncoderSim,
)


class Wrist(Subsystem):
    kP: float = 2.5
    kI: float = 0
    kD: float = 0.3

    kG: float = 1.685
    kS: float = 0

    tolerance: degrees = 2

    max_velocity: degrees_per_second = 90
    max_acceleration: degrees_per_second_squared = 180

    current_limit: amperes = 60

    min_angle: Rotation2d = Rotation2d.fromDegrees(-70)
    max_angle: Rotation2d = Rotation2d.fromDegrees(90)

    gearing: float = 25
    # SIMULATION ONLY
    mass: kilograms = 2.5
    length: meters = 0.75

    def __init__(self):
        self.nettable = NetworkTableInstance.getDefault().getTable("000Wrist")
        self.setName("000Wrist")

        self.motor = SparkMax(20, SparkMax.MotorType.kBrushless)

        self.encoder = self.motor.getAbsoluteEncoder()

        motor_config = SparkBaseConfig()
        motor_config.setIdleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(
            self.current_limit
        )
        motor_config.absoluteEncoder.positionConversionFactor(
            360
        ).velocityConversionFactor(360 * 60).zeroCentered(True).zeroOffset(
            (360 - 115) / 360
        )

        motor_config.closedLoop.P(self.kP).I(self.kI).D(self.kD).FeedbackSensor(
            motor_config.closedLoop.FeedbackSensor.kAbsoluteEncoder
        )
        """.maxMotion.maxVelocity(
            self.max_velocity
        ).maxAcceleration(self.max_acceleration).positionMode(
            MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal
        )
        """

        self.motor.configure(
            motor_config,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        self.closed_loop = self.motor.getClosedLoopController()

        self.setpoint = self.get_angle()

        self.feedforward = ArmFeedforward(self.kS, self.kG, 0)

        self.mech = Mechanism2d(220 * self.length, 220 * self.length)
        self.mech_root = self.mech.getRoot(
            "000Wrist", 110 * self.length, 110 * self.length
        )
        self.mech_lig = self.mech_root.appendLigament(
            "wrist", 100 * self.length, self.get_angle().degrees()
        )

        if RobotBase.isSimulation():
            box = DCMotor.NEO()
            self.spark_sim = SparkMaxSim(self.motor, box)
            self.encoder_sim = self.spark_sim.getAbsoluteEncoderSim()
            self.sim = SingleJointedArmSim(
                box,
                self.gearing,
                moi := SingleJointedArmSim.estimateMOI(self.length, self.mass),
                self.length,
                self.min_angle.radians(),
                self.max_angle.radians(),
                True,
                0,
                # self.get_angle().radians(),
            )
            self.nettable.putNumber("moi", moi)

        SmartDashboard.putData(self)
        SmartDashboard.putData("Wrist Mech", self.mech)

    def periodic(self) -> None:
        self.nettable.putNumber("Setpoint/degrees", self.setpoint.degrees())
        self.nettable.putNumber("Setpoint/radians", self.setpoint.radians())
        self.nettable.putNumber("Setpoint/rotations", self.setpoint.degrees() / 360)

        angle = self.get_angle()

        self.nettable.putNumber("Position/degrees", angle.degrees())
        self.nettable.putNumber("Position/radians", angle.radians())
        self.nettable.putNumber("Position/rotations", angle.degrees() / 360)

        self.nettable.putNumber("Error/degrees", (self.setpoint - angle).degrees())
        self.nettable.putNumber("Error/radians", (self.setpoint - angle).radians())
        self.nettable.putNumber(
            "Error/rotations", (self.setpoint - angle).degrees() / 360
        )

        self.mech_lig.setAngle(angle.degrees())

        ff = self.feedforward.calculate(
            self.get_angle().radians(), self.get_velocity().radians()
        )
        self.closed_loop.setReference(
            self.setpoint.degrees() / 360,
            SparkMax.ControlType.kPosition,
            arbFeedforward=ff,
        )

        self.nettable.putNumber("Current", self.motor.getOutputCurrent())
        self.nettable.putNumber("Output %", self.motor.getAppliedOutput())

    def simulationPeriodic(self) -> None:
        self.sim.update(0.02)

        self.spark_sim.iterate(
            (self.sim.getVelocityDps() / 360) * 60,
            RoboRioSim.getVInVoltage(),
            0.02,
        )

        self.encoder_sim.setPosition(self.sim.getAngleDegrees())
        self.encoder_sim.setVelocity(self.sim.getVelocityDps())

        RoboRioSim.setVInCurrent(self.motor.getOutputCurrent())

        self.sim.setInputVoltage(
            RoboRioSim.getVInVoltage() * self.motor.getAppliedOutput()
        )

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.encoder.getPosition())

    def get_velocity(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.encoder.getVelocity())

    def get_setpoint(self) -> Rotation2d:
        return self.setpoint

    def at_setpoint(self) -> bool:
        return abs((self.setpoint - self.get_angle()).degrees()) < self.tolerance

    def set_setpoint(self, setpoint: Rotation2d) -> None:
        if setpoint.radians() < self.min_angle.radians():
            setpoint = self.min_angle

        if setpoint.radians() > self.max_angle.radians():
            setpoint = self.max_angle
        self.setpoint = setpoint
