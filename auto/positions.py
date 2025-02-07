from wpimath.geometry import Pose2d, Rotation2d

FIELD_WIDTH_FT = 26 + 5 / 12
FIELD_LENGTH_FT = 57 + (6 + 7 / 8) / 12
ROBOT_WIDTH_FT = (
    28 / 12
)  # this file assumes the robot as being perfectly square for less complication
# this is not the length of the side, but rather the center to center distance for pegs
REEF_CENTER_DISTANCE_FT = 1 + 1 / 12
CORAL_STATION_X_LENGTH_FT = 67.851 / 12
CORAL_STATION_Y_LENGTH_FT = 51 / 12
CORAL_STATION_LEFT_ANGLE = Rotation2d.fromDegrees(135)
CORAL_STATION_RIGHT_ANGLE = -Rotation2d.fromDegrees(135)

reef_center = Pose2d.fromFeet(
    (144 + 93.5 / 2 + 14) / 12, FIELD_WIDTH_FT / 2, Rotation2d(0)
)

blue_reef_without_offset = Pose2d.fromFeet(
    144 / 12,
    FIELD_WIDTH_FT / 2,
    Rotation2d.fromDegrees(0),
)


def rotate_about_reef(pose: Pose2d, angle: Rotation2d) -> Pose2d:
    p = (
        pose.relativeTo(reef_center)
        .rotateBy(angle)
        .relativeTo(Pose2d(-reef_center.X(), -reef_center.Y(), Rotation2d(0)))
    )
    # for x, cos is parallel to the edge of the reef and sin will be perpendicular
    # for y, sin is parallel to the edge of the reef and cos will be perpendicular
    if angle.radians() < 0:
        return Pose2d.fromFeet(
            p.x_feet
            - REEF_CENTER_DISTANCE_FT * angle.cos()
            + ROBOT_WIDTH_FT * angle.sin() / 2,
            p.y_feet
            + REEF_CENTER_DISTANCE_FT * angle.sin()
            + ROBOT_WIDTH_FT * angle.cos() / 2,
            p.rotation(),
        )
    else:
        return Pose2d.fromFeet(
            p.x_feet
            + REEF_CENTER_DISTANCE_FT * angle.cos()
            - ROBOT_WIDTH_FT * angle.sin() / 2,
            p.y_feet
            - REEF_CENTER_DISTANCE_FT * angle.sin()
            + ROBOT_WIDTH_FT * angle.cos() / 2,
            p.rotation(),
        )


def rotate_about_center(pose: Pose2d) -> Pose2d:
    return pose.rotateBy(Rotation2d.fromDegrees(180)).relativeTo(
        Pose2d.fromFeet(-FIELD_LENGTH_FT / 2, -FIELD_WIDTH_FT / 2, Rotation2d(0))
    )


blue_start_line_left = Pose2d.fromFeet(
    325.5 / 12 - ROBOT_WIDTH_FT, FIELD_WIDTH_FT - 31.178 / 12, Rotation2d.fromDegrees(0)
)

blue_start_line_right = Pose2d.fromFeet(
    325.5 / 12 - ROBOT_WIDTH_FT, 31.178 / 12, Rotation2d.fromDegrees(0)
)

blue_start_line_center = Pose2d.fromFeet(
    325.5 / 12 - ROBOT_WIDTH_FT, 0, Rotation2d.fromDegrees(0)
)

blue_reef_a = Pose2d.fromFeet(
    144 / 12 - ROBOT_WIDTH_FT,
    FIELD_WIDTH_FT / 2 + REEF_CENTER_DISTANCE_FT / 2,
    Rotation2d.fromDegrees(0),
)

blue_reef_b = Pose2d.fromFeet(
    144 / 12 - ROBOT_WIDTH_FT,
    FIELD_WIDTH_FT / 2 - REEF_CENTER_DISTANCE_FT / 2,
    Rotation2d.fromDegrees(0),
)

blue_reef_h = Pose2d.fromFeet(
    (144 + 93.5 - 28) / 12 + ROBOT_WIDTH_FT,
    FIELD_WIDTH_FT / 2 + REEF_CENTER_DISTANCE_FT / 2,
    Rotation2d(180),
)

blue_reef_g = Pose2d.fromFeet(
    (144 + 93.5 - 28) / 12 + ROBOT_WIDTH_FT,
    FIELD_WIDTH_FT / 2 - REEF_CENTER_DISTANCE_FT / 2,
    Rotation2d(180),
)


blue_reef_l = rotate_about_reef(blue_reef_without_offset, Rotation2d.fromDegrees(-30))
blue_reef_k = rotate_about_reef(blue_reef_without_offset, Rotation2d.fromDegrees(-60))

blue_reef_j = rotate_about_reef(blue_reef_without_offset, Rotation2d.fromDegrees(-90))
blue_reef_i = rotate_about_reef(blue_reef_without_offset, Rotation2d.fromDegrees(-100))

blue_reef_c = rotate_about_reef(blue_reef_without_offset, Rotation2d.fromDegrees(45))
blue_reef_d = rotate_about_reef(blue_reef_without_offset, Rotation2d.fromDegrees(55))

blue_reef_e = rotate_about_reef(blue_reef_without_offset, Rotation2d.fromDegrees(60))
blue_reef_f = rotate_about_reef(blue_reef_without_offset, Rotation2d.fromDegrees(90))

blue_coral_intake_left_left = Pose2d.fromFeet(
    CORAL_STATION_X_LENGTH_FT
    - ROBOT_WIDTH_FT / 2
    - ROBOT_WIDTH_FT * CORAL_STATION_LEFT_ANGLE.cos(),
    FIELD_WIDTH_FT
    - ROBOT_WIDTH_FT / 2
    - ROBOT_WIDTH_FT * CORAL_STATION_LEFT_ANGLE.sin(),
    CORAL_STATION_LEFT_ANGLE,
)

blue_coral_intake_left_center = Pose2d.fromFeet(
    CORAL_STATION_X_LENGTH_FT / 2
    - ROBOT_WIDTH_FT / 2
    - ROBOT_WIDTH_FT * CORAL_STATION_LEFT_ANGLE.cos(),
    FIELD_WIDTH_FT
    - ROBOT_WIDTH_FT * CORAL_STATION_LEFT_ANGLE.sin()
    - CORAL_STATION_Y_LENGTH_FT / 2,
    CORAL_STATION_LEFT_ANGLE,
)

blue_coral_intake_left_right = Pose2d.fromFeet(
    ROBOT_WIDTH_FT / 2,
    FIELD_WIDTH_FT
    - ROBOT_WIDTH_FT * CORAL_STATION_LEFT_ANGLE.sin()
    - CORAL_STATION_Y_LENGTH_FT / 2,
    CORAL_STATION_LEFT_ANGLE,
)

blue_coral_intake_right_left = Pose2d.fromFeet(
    ROBOT_WIDTH_FT / 2 - ROBOT_WIDTH_FT * CORAL_STATION_LEFT_ANGLE.cos(),
    CORAL_STATION_X_LENGTH_FT,
    CORAL_STATION_LEFT_ANGLE,
)

blue_coral_intake_right_center = Pose2d.fromFeet(
    CORAL_STATION_X_LENGTH_FT / 2
    - ROBOT_WIDTH_FT / 2
    - ROBOT_WIDTH_FT * CORAL_STATION_LEFT_ANGLE.cos(),
    ROBOT_WIDTH_FT * CORAL_STATION_LEFT_ANGLE.sin() + CORAL_STATION_Y_LENGTH_FT / 2,
    CORAL_STATION_LEFT_ANGLE,
)

blue_coral_intake_right_right = Pose2d.fromFeet(
    CORAL_STATION_Y_LENGTH_FT + ROBOT_WIDTH_FT / 2,
    ROBOT_WIDTH_FT * CORAL_STATION_LEFT_ANGLE.sin() + CORAL_STATION_Y_LENGTH_FT / 2,
    CORAL_STATION_LEFT_ANGLE,
)

blue_processor = Pose2d.fromFeet()


red_
