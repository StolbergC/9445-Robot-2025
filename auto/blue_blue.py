from commands2 import SequentialCommandGroup, WaitCommand
from subsystems.drivetrain import Drivetrain

from wpimath.geometry import Pose2d, Rotation2d
from auto import positions


def get_auto(drivetrain: Drivetrain) -> SequentialCommandGroup:
    # start at starting line
    return (
        drivetrain.drive_position(positions.blue_start_line_left)
        .andThen(
            WaitCommand(0.25),
        )
        .andThen(drivetrain.drive_position(positions.blue_reef_i))
    )
