from commands2 import Subsystem

from wpilib import Mechanism2d, RobotBase, SmartDashboard, Timer
from wpilib.simulation import FlywheelSim, RoboRioSim

from wpimath.units import (
    amperes,
    kilogram_square_meters,
    lbsToKilograms,
    inchesToMeters,
    rotationsToDegrees,
)
from wpimath.controller import SimpleMotorFeedforwardRadians
from wpimath.system.plant import LinearSystemId, DCMotor
from wpimath.geometry import Rotation2d

from ntcore import NetworkTableInstance

from rev import SparkBaseConfig, SparkMax, SparkMaxSim


class Fingers(Subsystem):
    current_limit: amperes = 25
    slip_current_limit: amperes = 18
    left_inverted: bool = False

    kS: float = 0
    kV: float = 1 / 60
    kA: float = 0

    gearing: float = 9

    # moi of one wheel
    moi: kilogram_square_meters = lbsToKilograms(0.1) * inchesToMeters(1) * 0.75

    def __init__(self):
        self.nettable = NetworkTableInstance.getDefault().getTable("000Fingers")

        self.motor = SparkMax(28, SparkMax.MotorType.kBrushless)

        motor_config = (
            SparkBaseConfig()
            .smartCurrentLimit(self.current_limit)
            .inverted(self.left_inverted)
            .setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        )
        motor_config.encoder.velocityConversionFactor(self.gearing)

        self.motor.configure(
            motor_config,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.setpoint = Rotation2d(0)

        self.encoder = self.motor.getEncoder()

        self.feedforward = SimpleMotorFeedforwardRadians(self.kS, self.kV, self.kA)

        if RobotBase.isSimulation():
            self.motor_l_sim = SparkMaxSim(self.motor, DCMotor.NEO550())
            self.encoder_sim = self.motor_l_sim.getRelativeEncoderSim()
            self.sim = FlywheelSim(
                LinearSystemId.flywheelSystem(
                    DCMotor.NEO550(2), 4 * self.moi, self.gearing
                ),
                DCMotor.NEO550(2),
            )

        # one wheel spinning to demonstrate motion
        self.mech = Mechanism2d(100, 100)
        self.mech_root = self.mech.getRoot("FingerWheel", 50, 50)
        self.lig = self.mech_root.appendLigament("FingerLigament", 30, 0)

        self.stall_timer = Timer()

        SmartDashboard.putData(self)
        SmartDashboard.putData("FingerMech", self.mech)

    def periodic(self) -> None:
        self.nettable.putNumber("Setpoint/RPM", self.setpoint.degrees() / 360)
        self.nettable.putNumber(
            "Setpoint/Degrees Per Second", self.setpoint.degrees() / 60
        )
        self.nettable.putNumber(
            "Setpoint/Radians Per Second", self.setpoint.radians() / 60
        )

        velocity = self.get_velocity()

        current_l = self.motor.getOutputCurrent()

        self.nettable.putNumber("Velocity/RPM", velocity.degrees() / 360)
        self.nettable.putNumber("Velocity/Degrees Per Second", velocity.degrees() / 60)
        self.nettable.putNumber("Velocity/Radians Per Second", velocity.radians() / 60)

        if current_l > self.slip_current_limit * 0.9:
            self.stall_timer.start()
        if self.stall_timer.isRunning() and (
            self.get_velocity().degrees() >= 5
            or current_l < self.slip_current_limit * 0.8
        ):
            self.stall_timer.stop()

        ff = self.feedforward.calculate(
            self.get_velocity().radians(), self.setpoint.radians() / self.gearing
        )
        self.motor.setVoltage(ff)
        if self.stall_timer.hasElapsed(0.5):
            self.motor.stopMotor()

        self.lig.setAngle(self.lig.getAngle() + velocity.degrees() / 60)

        self.nettable.putNumber("FF/Volts", ff)
        self.nettable.putNumber("Current/left", current_l)
        self.nettable.putNumber("Stalling/left", self.stall_timer.get())
        self.nettable.putBoolean("Stalling/isLeft", self.stall_timer.isRunning())

    def simulationPeriodic(self) -> None:
        self.sim.update(0.02)

        self.motor_l_sim.iterate(
            self.sim.getAngularVelocity() * 60, RoboRioSim.getVInVoltage(), 0.02
        )

        # RoboRioSim.setVInCurrent(
        #     self.motor_l.getOutputCurrent() + self.motor_r.getOutputCurrent()
        # )
        self.sim.setInputVoltage(
            self.motor_l_sim.getAppliedOutput() * RoboRioSim.getVInVoltage()
        )

    def get_velocity(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.encoder.getVelocity() * 360 / 60)

    def set_setpoint(self, setpoint: Rotation2d) -> None:
        """
        The setpoint should be in Rotation2d/minute as its unit
        """
        self.setpoint = setpoint

    def get_setpoint(self) -> Rotation2d:
        return self.setpoint
