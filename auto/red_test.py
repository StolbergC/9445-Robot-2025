from commands2 import SequentialCommandGroup, WaitCommand
from subsystems.drivetrain import Drivetrain

from auto import positions


def get_auto(drivetrain: Drivetrain) -> SequentialCommandGroup:
    return (
        drivetrain.drive_position(positions.red_start_line_left)
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_reef_g))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_reef_a))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_reef_h))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_reef_b))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_reef_i))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_reef_c))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_reef_j))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_reef_d))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_reef_k))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_reef_e))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_reef_l))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_reef_f))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_coral_intake_left_left))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_coral_intake_right_right))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_coral_intake_left_center))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_coral_intake_right_center))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_coral_intake_left_right))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_coral_intake_right_left))
        .andThen(WaitCommand(0.25))
        .andThen(drivetrain.drive_position(positions.red_processor))
        .andThen(WaitCommand(0.25))
    )
