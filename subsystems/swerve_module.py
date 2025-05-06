from enum import Enum
from math import pi

from commands2 import Subsystem, SwerveControllerCommand

from wpilib import RobotBase

from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.units import meters_per_second, meters
from wpimath.filter import SlewRateLimiter

from ntcore import NetworkTableInstance

from phoenix6.hardware import TalonFX
from phoenix6.hardware.cancoder import CANcoder
from phoenix6.configs import FeedbackConfigs, MagnetSensorConfigs
from phoenix6.controls import VelocityDutyCycle, PositionDutyCycle
from phoenix6.signals import FeedbackSensorSourceValue, NeutralModeValue, InvertedValue

from constants import ModuleConstants
import constants
from errors import Error


class ModuleLocation(Enum):
    FRONT_LEFT = (0,)
    FRONT_RIGHT = (1,)
    BACK_LEFT = (2,)
    BACK_RIGHT = (3,)


class SwerveModule(Subsystem):
    def __init__(self, location: ModuleLocation):
        super().__init__()
        swerve_consts = constants

        self.consts: ModuleConstants = None
        if location == ModuleLocation.FRONT_LEFT:
            self.consts = swerve_consts.front_left
            self.setName("Swerve Module FL")
        elif location == ModuleLocation.FRONT_RIGHT:
            self.consts = swerve_consts.front_right
            self.setName("Swerve Module FR")
        elif location == ModuleLocation.BACK_LEFT:
            self.consts = swerve_consts.back_left
            self.setName("Swerve Module BL")
        elif location == ModuleLocation.BACK_RIGHT:
            self.consts = swerve_consts.back_right
            self.setName("Swerve Module BR")
        else:
            Error(
                ValueError(
                    f"There was an invalid module location given to SwerveModule {location}"
                )
            )
            # this may be bad. If in a real match, the code will report an error, then keep running. What is the consts value there?
            self.consts = swerve_consts.front_left
            self.setName("Swerve Module ERROR")

        self.nettable = NetworkTableInstance.getDefault().getTable(
            f"/swerve/modules/{self.getName()}"
        )

        consts = self.consts

        self.drive_motor = TalonFX(consts.drive_id, constants.canbus)
        self.turn_motor = TalonFX(consts.turn_id, constants.canbus)
        self.cancoder = CANcoder(consts.cancoder_id, constants.canbus)

        # if RobotBase.isSimulation():
        # self.cancoder.configurator.apply(swerve_consts.cancoder_config)
        # else:
        self.cancoder.configurator.apply(
            swerve_consts.cancoder_config.with_magnet_sensor(
                MagnetSensorConfigs().with_magnet_offset(consts.cancoder_offset)
            )
        )
        if not consts.drive_inverted:
            self.drive_motor.configurator.apply(swerve_consts.drive_config)
        else:
            if (
                swerve_consts.drive_config.motor_output.inverted
                == InvertedValue.CLOCKWISE_POSITIVE
            ):
                new_inversion = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            else:
                new_inversion = InvertedValue.CLOCKWISE_POSITIVE
            self.drive_motor.configurator.apply(
                swerve_consts.drive_config.with_motor_output(
                    swerve_consts.drive_config.motor_output.with_inverted(new_inversion)
                )
            )

        self.turn_motor.configurator.apply(
            swerve_consts.turn_config.with_feedback(
                FeedbackConfigs()
                .with_feedback_remote_sensor_id(consts.cancoder_id)
                .with_feedback_sensor_source(FeedbackSensorSourceValue.FUSED_CANCODER)
            )
        )

        self.cancoder.set_position(self.cancoder.get_absolute_position().value)

        self.setpoint = SwerveModuleState()
        try_angle = self.turn_motor.get_position()
        try_speed = self.drive_motor.get_velocity()
        i = 0
        while not try_speed.is_all_good and not try_angle.is_all_good and i < 10:
            try_angle = self.turn_motor.get_position()
            try_speed = self.drive_motor.get_velocity()
            i += 1
        self.last_good_angle = try_angle.value

        self.last_good_speed = try_speed.value

        self.commanded_pub = self.nettable.getStructTopic(
            "State/Commanded", SwerveModuleState
        ).publish()
        self.actual_pub = self.nettable.getStructTopic(
            "State/Actual", SwerveModuleState
        ).publish()

        self.set_state(self.setpoint)

        self.set_drive_idle(False)
        self.set_turn_idle(False)

        if RobotBase.isSimulation():
            self.cancoder_sim = self.cancoder.sim_state
            self.drive_limiter = SlewRateLimiter(500)
            self.turn_limiter = SlewRateLimiter(500)

    def periodic(self):
        # self.setpoint.cosineScale(self.get_angle())
        # self.setpoint.optimize(self.get_angle())
        if abs(self.setpoint.speed) < 0.025:
            self.drive_motor.set(0)
        else:
            self.drive_motor.set_control(
                VelocityDutyCycle(self.meters_to_rotations(-self.setpoint.speed))
            )

        self.turn_motor.set_control(
            PositionDutyCycle(
                self.setpoint.angle.degrees() / 360  # constants.turn_ratio
            )
        )

        self.actual_pub.set(
            SwerveModuleState(
                self.get_speed(),
                self.get_angle(),
            ),
        )

        self.nettable.putNumber("State/Drive Out", self.drive_motor.get())
        self.nettable.putNumber("State/Turn Out", self.turn_motor.get())
        self.nettable.putNumber(
            "State/Turn Velocity", self.turn_motor.get_velocity().value
        )
        self.nettable.putNumber("State/drive distance", self.get_distance())
        self.nettable.putNumber(
            "State/Velocity Setpoint",
            self.drive_motor.get_closed_loop_reference().value,
        )

        self.nettable.putNumber(
            "State/turn error", self.turn_motor.get_closed_loop_error().value
        )

        self.nettable.putNumber(
            "State/turn reference", self.turn_motor.get_closed_loop_reference().value
        )

        self.nettable.putNumber(
            "State/drive reference", self.drive_motor.get_closed_loop_reference().value
        )

        # return super().periodic()

    def simulationPeriodic(self):
        # Drive Motor Position and Velocity
        driveRps = self.drive_limiter.calculate(100 * self.drive_motor.get())
        # driveRps = self.drive_motor.get_velocity().value
        self.drive_motor.sim_state.set_rotor_velocity(driveRps)
        self.drive_motor.sim_state.add_rotor_position(driveRps * 0.02)

        # Turn Motor Position and Velocity
        turnRps = self.turn_limiter.calculate(100 * self.turn_motor.get())
        # turnRps = 6000
        # turnRps = self.turn_motor.get_velocity().value * SwerveConstants.turn_ratio
        self.turn_motor.sim_state.set_rotor_velocity(turnRps)
        self.turn_motor.sim_state.add_rotor_position(turnRps * 0.02)

        # CANcoder Velocity and Position
        canRps = turnRps * constants.turn_ratio
        self.cancoder_sim.set_velocity(canRps)
        self.cancoder_sim.add_position(canRps * 0.02)
        self.nettable.putNumber("CanTurnRPS", canRps)

    def get_speed(self) -> meters_per_second:
        try_speed = self.drive_motor.get_velocity()
        i = 0
        while not try_speed.is_all_good and i < 10:
            try_speed = self.drive_motor.get_velocity()
            i += 1
        self.last_good_speed = try_speed.value
        return self.rotations_to_meters(try_speed.value)

    def get_angle(self) -> Rotation2d:
        try_angle = self.cancoder.get_absolute_position()  # .wait_for_update(0.1)
        if try_angle.is_all_good():
            self.last_good_angle = try_angle.value
            return Rotation2d.fromRotations(self.turn_motor.get_position().value)
        else:
            return Rotation2d.fromRotations(self.last_good_angle)

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_speed(), self.get_angle())

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.get_distance(), self.get_angle())

    def get_distance(self) -> meters:
        return self.rotations_to_meters(self.drive_motor.get_position().value)

    def set_state(self, state: SwerveModuleState) -> None:
        state.optimize(self.get_angle())
        state.cosineScale(self.get_angle())

        self.setpoint = state
        self.commanded_pub.set(state)

    def set_drive_idle(self, coast: bool) -> None:
        self.drive_motor.setNeutralMode(
            NeutralModeValue.COAST if coast else NeutralModeValue.BRAKE
        )

    def set_turn_idle(self, coast: bool) -> None:
        self.turn_motor.setNeutralMode(
            NeutralModeValue.COAST if coast else NeutralModeValue.BRAKE
        )

    def rotations_to_meters(self, rotations: float) -> meters:
        return SwerveModule.rotations_to_meters(rotations)

    def meters_to_rotations(self, m: meters) -> float:
        return SwerveModule.meters_to_rotations(m)

    @staticmethod
    def rotations_to_meters(rotations: float) -> meters:
        return rotations * constants.drive_ratio * constants.wheel_radius * 2 * pi

    @staticmethod
    def meters_to_rotations(m: meters) -> float:
        return m / (constants.drive_ratio * constants.wheel_radius * 2 * pi)

    def get_from_center(self) -> Translation2d:
        return self.consts.translation_from_center
