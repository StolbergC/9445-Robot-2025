from wpimath.geometry import Pose2d, Rotation2d

FIELD_WIDTH_FT = 26 + 5 / 12
FIELD_LENGTH_FT = 57 + (6 + 7 / 8) / 12
ROBOT_WIDTH_FT = (
    28 / 12 + 0.5
)  # this file assumes the robot as being perfectly square for less complication
# this is not the length of the side, but rather the center to center distance for pegs
REEF_CENTER_DISTANCE_FT = 1 + 1 / 12
CORAL_STATION_X_LENGTH_FT = 67.851 / 12
CORAL_STATION_Y_LENGTH_FT = 51 / 12
CORAL_STATION_LENGTH_FT = 77.750 / 12
CORAL_STATION_LEFT_ANGLE = Rotation2d(64.528, 46.863) + Rotation2d.fromDegrees(90)
CORAL_STATION_RIGHT_ANGLE = -CORAL_STATION_LEFT_ANGLE

blue_reef_center = Pose2d.fromFeet(
    (144 + 93.5 / 2) / 12 - ROBOT_WIDTH_FT / 2, FIELD_WIDTH_FT / 2, Rotation2d(0)
)

blue_reef_without_offset = Pose2d.fromFeet(
    144 / 12,
    FIELD_WIDTH_FT / 2,
    Rotation2d.fromDegrees(0),
)


def rotate_about_reef(pose: Pose2d, angle: Rotation2d) -> Pose2d:
    return (
        pose.relativeTo(blue_reef_center)
        .rotateBy(angle)
        .relativeTo(Pose2d(-blue_reef_center.X(), -blue_reef_center.Y(), Rotation2d(0)))
    )


def rotate_about_center(pose: Pose2d) -> Pose2d:
    return pose.rotateBy(Rotation2d.fromDegrees(180)).relativeTo(
        Pose2d.fromFeet(-FIELD_LENGTH_FT, -FIELD_WIDTH_FT, Rotation2d(0))
    )


blue_reef_a = Pose2d.fromFeet(
    144 / 12 - ROBOT_WIDTH_FT - 1.5,
    FIELD_WIDTH_FT / 2 + REEF_CENTER_DISTANCE_FT / 2,
    Rotation2d.fromDegrees(0),
)

blue_reef_a_far = Pose2d.fromFeet(
    blue_reef_a.x_feet - ROBOT_WIDTH_FT,
    blue_reef_a.y_feet,
    blue_reef_a.rotation(),
)

blue_reef_b = Pose2d.fromFeet(
    144 / 12 - ROBOT_WIDTH_FT - 1.5,
    FIELD_WIDTH_FT / 2 - REEF_CENTER_DISTANCE_FT / 2,
    Rotation2d.fromDegrees(0),
)

blue_reef_b_far = Pose2d.fromFeet(
    blue_reef_b.x_feet - ROBOT_WIDTH_FT,
    blue_reef_b.y_feet,
    blue_reef_b.rotation(),
)

blue_reef_h = rotate_about_reef(blue_reef_a, Rotation2d.fromDegrees(180))
blue_reef_g = rotate_about_reef(blue_reef_b, Rotation2d.fromDegrees(180))

blue_reef_h_far = rotate_about_reef(blue_reef_a_far, Rotation2d.fromDegrees(180))
blue_reef_g_far = rotate_about_reef(blue_reef_b_far, Rotation2d.fromDegrees(180))

# blue_reef_h = Pose2d.fromFeet(
#     (144 + 93.5 - 28) / 12 + ROBOT_WIDTH_FT / 2,
#     FIELD_WIDTH_FT / 2 + REEF_CENTER_DISTANCE_FT / 2,
#     Rotation2d(180),
# )

# blue_reef_g = Pose2d.fromFeet(
#     (144 + 93.5 - 28) / 12 + ROBOT_WIDTH_FT / 2,
#     FIELD_WIDTH_FT / 2 - REEF_CENTER_DISTANCE_FT / 2,
#     Rotation2d(180),
# )


blue_reef_k = rotate_about_reef(blue_reef_a, Rotation2d.fromDegrees(-60))
blue_reef_l = rotate_about_reef(blue_reef_b, Rotation2d.fromDegrees(-60))

blue_reef_k_far = rotate_about_reef(blue_reef_a_far, Rotation2d.fromDegrees(-60))
blue_reef_l_far = rotate_about_reef(blue_reef_b_far, Rotation2d.fromDegrees(-60))

blue_reef_i = rotate_about_reef(blue_reef_a, Rotation2d.fromDegrees(-120))
blue_reef_j = rotate_about_reef(blue_reef_b, Rotation2d.fromDegrees(-120))

blue_reef_i_far = rotate_about_reef(blue_reef_a_far, Rotation2d.fromDegrees(-120))
blue_reef_j_far = rotate_about_reef(blue_reef_b_far, Rotation2d.fromDegrees(-120))

blue_reef_c = rotate_about_reef(blue_reef_a, Rotation2d.fromDegrees(60))
blue_reef_d = rotate_about_reef(blue_reef_b, Rotation2d.fromDegrees(60))

blue_reef_c_far = rotate_about_reef(blue_reef_a_far, Rotation2d.fromDegrees(60))
blue_reef_d_far = rotate_about_reef(blue_reef_b_far, Rotation2d.fromDegrees(60))

blue_reef_e = rotate_about_reef(blue_reef_a, Rotation2d.fromDegrees(120))
blue_reef_f = rotate_about_reef(blue_reef_b, Rotation2d.fromDegrees(120))

blue_reef_e_far = rotate_about_reef(blue_reef_a_far, Rotation2d.fromDegrees(120))
blue_reef_f_far = rotate_about_reef(blue_reef_b_far, Rotation2d.fromDegrees(120))

blue_algae_ab = Pose2d.fromFeet(
    144 / 12 - ROBOT_WIDTH_FT / 2,
    FIELD_WIDTH_FT / 2,
    Rotation2d.fromDegrees(0),
)

blue_algae_ab_far = Pose2d.fromFeet(
    blue_algae_ab.x_feet - ROBOT_WIDTH_FT,
    blue_algae_ab.y_feet,
    blue_algae_ab.rotation(),
)

blue_algae_kl = rotate_about_reef(blue_algae_ab, Rotation2d.fromDegrees(-60))
blue_algae_ij = rotate_about_reef(blue_algae_ab, Rotation2d.fromDegrees(-120))
blue_algae_cd = rotate_about_reef(blue_algae_ab, Rotation2d.fromDegrees(60))
blue_algae_ef = rotate_about_reef(blue_algae_ab, Rotation2d.fromDegrees(120))
blue_algae_gh = rotate_about_reef(blue_algae_ab, Rotation2d.fromDegrees(180))

blue_algae_kl_far = rotate_about_reef(blue_algae_ab_far, Rotation2d.fromDegrees(-60))
blue_algae_ij_far = rotate_about_reef(blue_algae_ab_far, Rotation2d.fromDegrees(-120))
blue_algae_cd_far = rotate_about_reef(blue_algae_ab_far, Rotation2d.fromDegrees(60))
blue_algae_ef_far = rotate_about_reef(blue_algae_ab_far, Rotation2d.fromDegrees(120))
blue_algae_gh_far = rotate_about_reef(blue_algae_ab_far, Rotation2d.fromDegrees(180))

blue_coral_intake_left_left = Pose2d.fromFeet(
    -ROBOT_WIDTH_FT / 2
    - CORAL_STATION_LENGTH_FT * CORAL_STATION_LEFT_ANGLE.cos()
    + ROBOT_WIDTH_FT * CORAL_STATION_LEFT_ANGLE.sin()
    - 0.5,
    FIELD_WIDTH_FT
    - ROBOT_WIDTH_FT / 2
    + ROBOT_WIDTH_FT * CORAL_STATION_LEFT_ANGLE.cos()
    - 0.5,
    CORAL_STATION_LEFT_ANGLE,
)

blue_coral_intake_left_center = Pose2d.fromFeet(
    -ROBOT_WIDTH_FT / 2
    - 4 / 3 * CORAL_STATION_LENGTH_FT * CORAL_STATION_LEFT_ANGLE.cos(),
    FIELD_WIDTH_FT
    - CORAL_STATION_LENGTH_FT * CORAL_STATION_LEFT_ANGLE.sin()
    + ROBOT_WIDTH_FT / 2,
    CORAL_STATION_LEFT_ANGLE,
)

blue_coral_intake_left_right = Pose2d.fromFeet(
    ROBOT_WIDTH_FT / 2,
    FIELD_WIDTH_FT - CORAL_STATION_LENGTH_FT * CORAL_STATION_LEFT_ANGLE.sin(),
    CORAL_STATION_LEFT_ANGLE,
)

blue_coral_intake_right_left = Pose2d.fromFeet(
    ROBOT_WIDTH_FT / 2,
    CORAL_STATION_X_LENGTH_FT,
    CORAL_STATION_LEFT_ANGLE,
)

blue_coral_intake_right_center = Pose2d.fromFeet(
    CORAL_STATION_Y_LENGTH_FT + ROBOT_WIDTH_FT / 2 * CORAL_STATION_RIGHT_ANGLE.cos(),
    ROBOT_WIDTH_FT * CORAL_STATION_LEFT_ANGLE.sin() + CORAL_STATION_Y_LENGTH_FT / 2,
    CORAL_STATION_RIGHT_ANGLE,
)

blue_coral_intake_right_right = Pose2d.fromFeet(
    CORAL_STATION_Y_LENGTH_FT
    + ROBOT_WIDTH_FT / 2
    + CORAL_STATION_RIGHT_ANGLE.sin() / 2,
    CORAL_STATION_LENGTH_FT * CORAL_STATION_LEFT_ANGLE.sin()
    - CORAL_STATION_Y_LENGTH_FT / 2,
    CORAL_STATION_RIGHT_ANGLE,
)

blue_processor = Pose2d.fromFeet(
    325.5 / 12 - ROBOT_WIDTH_FT - 61.76 / 12,
    ROBOT_WIDTH_FT / 2,
    Rotation2d.fromDegrees(-90),
)

red_reef_a = rotate_about_center(blue_reef_a)
red_reef_b = rotate_about_center(blue_reef_b)

red_reef_a_far = rotate_about_center(blue_reef_a_far)
red_reef_b_far = rotate_about_center(blue_reef_b_far)

red_reef_h = rotate_about_center(blue_reef_h)
red_reef_g = rotate_about_center(blue_reef_g)

red_reef_h_far = rotate_about_center(blue_reef_h_far)
red_reef_g_far = rotate_about_center(blue_reef_g_far)

red_reef_l = rotate_about_center(blue_reef_l)
red_reef_k = rotate_about_center(blue_reef_k)

red_reef_l_far = rotate_about_center(blue_reef_l_far)
red_reef_k_far = rotate_about_center(blue_reef_k_far)

red_reef_j = rotate_about_center(blue_reef_j)
red_reef_i = rotate_about_center(blue_reef_i)

red_reef_j_far = rotate_about_center(blue_reef_j_far)
red_reef_i_far = rotate_about_center(blue_reef_i_far)

red_reef_c = rotate_about_center(blue_reef_c)
red_reef_d = rotate_about_center(blue_reef_d)

red_reef_c_far = rotate_about_center(blue_reef_c_far)
red_reef_d_far = rotate_about_center(blue_reef_d_far)

red_reef_e = rotate_about_center(blue_reef_e)
red_reef_f = rotate_about_center(blue_reef_f)

red_reef_e_far = rotate_about_center(blue_reef_e_far)
red_reef_f_far = rotate_about_center(blue_reef_f_far)

red_coral_intake_left_left = rotate_about_center(blue_coral_intake_left_left)

red_coral_intake_left_center = rotate_about_center(blue_coral_intake_left_center)

red_coral_intake_left_right = rotate_about_center(blue_coral_intake_left_right)

red_coral_intake_right_left = rotate_about_center(blue_coral_intake_right_left)

red_coral_intake_right_center = rotate_about_center(blue_coral_intake_right_center)

red_coral_intake_right_right = rotate_about_center(blue_coral_intake_right_right)

red_processor = rotate_about_center(blue_processor)

red_algae_ab = rotate_about_center(blue_algae_ab)
red_algae_kl = rotate_about_center(blue_algae_kl)
red_algae_ij = rotate_about_center(blue_algae_ij)
red_algae_cd = rotate_about_center(blue_algae_cd)
red_algae_ef = rotate_about_center(blue_algae_ef)
red_algae_gh = rotate_about_center(blue_algae_gh)

red_algae_ab_far = rotate_about_center(blue_algae_ab_far)
red_algae_kl_far = rotate_about_center(blue_algae_kl_far)
red_algae_ij_far = rotate_about_center(blue_algae_ij_far)
red_algae_cd_far = rotate_about_center(blue_algae_cd_far)
red_algae_ef_far = rotate_about_center(blue_algae_ef_far)
red_algae_gh_far = rotate_about_center(blue_algae_gh_far)


blue_start_line_left = Pose2d.fromFeet(
    325.5 / 12 - ROBOT_WIDTH_FT,
    # FIELD_WIDTH_FT - 31.178 / 12,
    blue_reef_j.y_feet,
    # Rotation2d.fromDegrees(180),
    blue_reef_j.rotation(),
)

blue_start_line_right = Pose2d.fromFeet(
    325.5 / 12 - ROBOT_WIDTH_FT,
    # 31.178 / 12,
    blue_reef_e.y_feet,
    # Rotation2d.fromDegrees(180),
    blue_reef_e.rotation(),
)

blue_start_line_center = Pose2d.fromFeet(
    325.5 / 12 - ROBOT_WIDTH_FT, 0, Rotation2d.fromDegrees(180)
)

red_start_line_left = rotate_about_center(blue_start_line_left)

red_start_line_right = rotate_about_center(blue_start_line_right)

red_start_line_center = rotate_about_center(blue_start_line_center)
