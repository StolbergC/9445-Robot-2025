from subsystems.drivetrain import Drivetrain

from wpimath.geometry import Pose2d, Rotation2d

from commands2 import WaitCommand, SequentialCommandGroup


def generate_path(
    drivetrain: Drivetrain, positions: list[Pose2d]
) -> SequentialCommandGroup:
    out = WaitCommand(0)
    return SequentialCommandGroup(
        *[
            drivetrain.drive_position(position).andThen(WaitCommand(0.5))
            for position in positions
        ]
    )
