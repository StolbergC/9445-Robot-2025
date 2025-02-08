from commands2 import SequentialCommandGroup, WaitCommand
from subsystems.drivetrain import Drivetrain

from auto import positions


def get_auto(drivetrain: Drivetrain) -> SequentialCommandGroup:
    return (
        drivetrain.reset_pose(positions.blue_start_line_left)
        .andThen(WaitCommand(0.1))
        .andThen(drivetrain.drive_position(positions.blue_reef_i))
        .andThen(WaitCommand(0.25))  # score
        .andThen(drivetrain.drive_position(positions.blue_coral_intake_left_left))
        .andThen(WaitCommand(0.25))  # intake
        .andThen(drivetrain.drive_position(positions.blue_reef_j))
        .andThen(WaitCommand(0.25))  # score
        .andThen(drivetrain.drive_position(positions.blue_coral_intake_left_left))
        .andThen(WaitCommand(0.25))  # intake
        .andThen(drivetrain.drive_position(positions.blue_reef_k))
        .andThen(WaitCommand(0.25))  # score
        .andThen(drivetrain.drive_position(positions.blue_coral_intake_left_left))
        .andThen(WaitCommand(0.25))  # intake
        .andThen(drivetrain.drive_position(positions.blue_reef_l))
    )
