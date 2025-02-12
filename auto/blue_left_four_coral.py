from commands2 import Command, WaitCommand, WrapperCommand
from subsystems.drivetrain import Drivetrain

from auto import positions


def get_auto(drivetrain: Drivetrain) -> WrapperCommand:
    # return (
    #     drivetrain.reset_pose(positions.blue_start_line_left)
    #     .andThen(WaitCommand(1.25))
    #     .andThen(drivetrain.drive_position(positions.blue_processor))
    #     .withName("Test")
    # )
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
        .withName("Blue Left Four Coral")
        .withInterruptBehavior(
            Command.InterruptionBehavior.kCancelIncoming
        )  # I do not know why this is needed, but auto does not run in the autonomousInit without it
    )
