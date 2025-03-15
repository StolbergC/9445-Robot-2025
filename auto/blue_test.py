from commands2 import Command, SequentialCommandGroup, WaitCommand
from subsystems.drivetrain import Drivetrain

from auto import positions


def get_auto(drivetrain: Drivetrain) -> SequentialCommandGroup:
    # start at starting line
    delay = 1
    return (
        # drivetrain.drive_position(positions.blue_start_line_left)
        # .andThen(WaitCommand(delay))
        # .andThen(drivetrain.drive_position(positions.blue_reef_g))
        # .andThen(WaitCommand(delay))
        # .andThen(drivetrain.drive_position(positions.blue_reef_a))
        # .andThen(WaitCommand(delay))
        # .andThen(drivetrain.drive_position(positions.blue_reef_h))
        # .andThen(WaitCommand(delay))
        # .andThen(drivetrain.drive_position(positions.blue_reef_b))
        # .andThen(WaitCommand(delay))
        # .andThen(drivetrain.drive_position(positions.blue_reef_i))
        # .andThen(WaitCommand(delay))
        # .andThen(drivetrain.drive_position(positions.blue_reef_c))
        # .andThen(WaitCommand(delay))
        # .andThen(drivetrain.drive_position(positions.blue_reef_j))
        # .andThen(WaitCommand(delay))
        # .andThen(drivetrain.drive_position(positions.blue_reef_d))
        # .andThen(WaitCommand(delay))
        # .andThen(drivetrain.drive_position(positions.blue_reef_k))
        # .andThen(WaitCommand(delay))
        # .andThen(drivetrain.drive_position(positions.blue_reef_e))
        # .andThen(WaitCommand(delay))
        # .andThen(drivetrain.drive_position(positions.blue_reef_l))
        # .andThen(WaitCommand(delay))
        # .andThen(drivetrain.drive_position(positions.blue_reef_f))
        # .andThen(WaitCommand(delay))
        # .andThen(drivetrain.drive_position(positions.blue_coral_intake_left_left))
        # .andThen(WaitCommand(delay))
        WaitCommand(0)
        .andThen(drivetrain.drive_position(positions.blue_coral_intake_right_right))
        .andThen(WaitCommand(delay))
        .andThen(drivetrain.drive_position(positions.blue_coral_intake_left_center))
        .andThen(WaitCommand(delay))
        .andThen(drivetrain.drive_position(positions.blue_coral_intake_right_center))
        .andThen(WaitCommand(delay))
        .andThen(drivetrain.drive_position(positions.blue_coral_intake_left_right))
        .andThen(WaitCommand(delay))
        .andThen(drivetrain.drive_position(positions.blue_coral_intake_right_left))
        .andThen(WaitCommand(delay))
        .andThen(drivetrain.drive_position(positions.blue_processor))
        .andThen(WaitCommand(delay))
    ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)


def reset_pose(drivetrain: Drivetrain) -> None:
    drivetrain.reset_pose(positions.blue_start_line_left)
