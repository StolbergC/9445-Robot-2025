from math import pi

from wpilib import RobotBase
from wpimath.units import (
    meters,
    inchesToMeters,
    meters_per_second,
    feetToMeters,
    radians_per_second,
    degreesToRadians,
    kilograms,
    lbsToKilograms,
    turns,
)
from wpimath.geometry import Translation2d

from ntcore.util import ntproperty

from phoenix6.configs import (
    TalonFXConfiguration,
    CANcoderConfiguration,
    CurrentLimitsConfigs,
    Slot0Configs,
    ClosedLoopGeneralConfigs,
    FeedbackConfigs,
    MagnetSensorConfigs,
    MotorOutputConfigs,
    ClosedLoopRampsConfigs,
)

from phoenix6.signals import (
    SensorDirectionValue,
    FeedbackSensorSourceValue,
    InvertedValue,
)


class ModuleConstants:
    drive_id: int
    turn_id: int
    cancoder_id: int
    cancoder_offset: turns
    translation_from_center: Translation2d
    drive_inverted: bool

    def __init__(
        self,
        drive: int,
        cancoder: int,
        turn: int,
        cancoder_offset: turns,
        translation: Translation2d,
        drive_inverted: bool,
    ):
        self.drive_id = drive
        self.turn_id = turn
        self.cancoder_id = cancoder
        self.cancoder_offset = cancoder_offset
        self.translation_from_center = translation
        self.drive_inverted = drive_inverted if RobotBase.isReal() else False


front_left: ModuleConstants = ModuleConstants(
    14, 15, 16, -0.416, Translation2d.fromFeet(12 / 12, 12 / 12), False
)
front_right: ModuleConstants = ModuleConstants(
    11, 12, 13, 0.951, Translation2d.fromFeet(12 / 12, -12 / 12), True
)
back_left: ModuleConstants = ModuleConstants(
    5, 6, 7, -0.519, Translation2d.fromFeet(-12 / 12, 12 / 12), True
)
back_right: ModuleConstants = ModuleConstants(
    8, 9, 10, 0.520, Translation2d.fromFeet(-12 / 12, -12 / 12), False
)

drive_ratio: float = 1 / 8.14 if RobotBase.isReal() else 8.14
turn_ratio: float = 7 / 150

drivebase_width: meters = inchesToMeters(28)
drivebase_length: meters = inchesToMeters(28)
mass: kilograms = lbsToKilograms(120)
wheel_radius: meters = inchesToMeters(2)

canbus: str = "canivore1"


# should not need to modify much beyond here
drive_config: TalonFXConfiguration = (
    TalonFXConfiguration()
    .with_current_limits(CurrentLimitsConfigs().with_stator_current_limit(60))
    .with_slot0(
        Slot0Configs()
        .with_k_p(0.00)
        .with_k_i(0.0)
        .with_k_d(0.00)
        .with_k_v(0.01)
        .with_k_a(0.0)
        if RobotBase.isReal()
        else Slot0Configs()
        .with_k_p(0.0003)
        .with_k_i(0.0)
        .with_k_d(0.00)
        .with_k_v(0.0098296)
        .with_k_a(0.05)
    )
    # .with_feedback(
    # FeedbackConfigs().with_sensor_to_mechanism_ratio(
    #     1 / (drive_ratio * wheel_radius * 2 * pi)
    # )
    # )
    .with_motor_output(
        MotorOutputConfigs().with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
    )
)

turn_config: TalonFXConfiguration = (
    TalonFXConfiguration()
    .with_current_limits(CurrentLimitsConfigs().with_stator_current_limit(20))
    .with_slot0(
        # simulation
        Slot0Configs()
        .with_k_p(0.6)
        .with_k_i(0.0)
        .with_k_d(0.0)
        .with_k_v(0.0)
        .with_k_a(0.0)
        if RobotBase.isSimulation()
        else Slot0Configs().with_k_p(5.0)
    )
    .with_feedback(
        FeedbackConfigs()
        .with_feedback_sensor_source(FeedbackSensorSourceValue.FUSED_CANCODER)
        .with_rotor_to_sensor_ratio(turn_ratio)
    )
    .with_closed_loop_general(ClosedLoopGeneralConfigs().with_continuous_wrap(True))
    .with_motor_output(
        MotorOutputConfigs().with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
    )
)


cancoder_config: CANcoderConfiguration = CANcoderConfiguration().with_magnet_sensor(
    MagnetSensorConfigs()
    .with_absolute_sensor_discontinuity_point(0.5)
    .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
)

# def __init__(self):
#     self.drive_config = TalonFXConfiguration()
#     self.drive_config.audio.
