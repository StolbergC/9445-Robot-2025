from math import pi

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

    def __init__(
        self,
        drive: int,
        turn: int,
        cancoder: int,
        cancoder_offset: turns,
        translation: Translation2d,
    ):
        self.drive_id = drive
        self.turn_id = turn
        self.cancoder_id = cancoder
        self.cancoder_offset = cancoder_offset
        self.translation_from_center = translation


front_left: ModuleConstants = ModuleConstants(
    1, 2, 3, 0, Translation2d.fromFeet(14 / 12, 14 / 12)
)
front_right: ModuleConstants = ModuleConstants(
    4, 5, 6, 0, Translation2d.fromFeet(14 / 12, -14 / 12)
)
back_left: ModuleConstants = ModuleConstants(
    7, 8, 9, 0, Translation2d.fromFeet(-14 / 12, 14 / 12)
)
back_right: ModuleConstants = ModuleConstants(
    10, 11, 12, 0, Translation2d.fromFeet(-14 / 12, -14 / 12)
)

drive_ratio: float = 1 / 1  # 8.14
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
        .with_k_p(0.008)
        .with_k_i(0.0)
        .with_k_d(0.00)
        .with_k_v(0.0)
        .with_k_a(0.0)
    )
    # .with_feedback(
    # FeedbackConfigs().with_sensor_to_mechanism_ratio(
    #     1 / (drive_ratio * wheel_radius * 2 * pi)
    # )
    # )
    .with_motor_output(
        MotorOutputConfigs().with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
    )
)

turn_config: TalonFXConfiguration = (
    TalonFXConfiguration()
    .with_current_limits(CurrentLimitsConfigs().with_stator_current_limit(20))
    .with_slot0(
        Slot0Configs()
        .with_k_p(1.0)
        .with_k_i(0.0)
        .with_k_d(0.1)
        # .with_k_v(0.0)
        # .with_k_a(0.0)
    )
    .with_feedback(
        FeedbackConfigs()
        .with_feedback_sensor_source(FeedbackSensorSourceValue.FUSED_CANCODER)
        .with_rotor_to_sensor_ratio(turn_ratio)
    )
    .with_closed_loop_general(ClosedLoopGeneralConfigs().with_continuous_wrap(True))
    .with_motor_output(
        MotorOutputConfigs().with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
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
