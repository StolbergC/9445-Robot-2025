from auto import positions
from subsystems.drivetrain import Drivetrain

from wpimath.geometry import Pose2d, Rotation2d
from commands2 import Command, DeferredCommand, RepeatCommand


def get_auto(drivetrain: Drivetrain):
    return (
        drivetrain.reset_pose(Pose2d.fromFeet(0, 0, Rotation2d.fromDegrees(0)))
        .andThen(
            RepeatCommand(
                DeferredCommand(
                    lambda: drivetrain.drive_joystick(
                        lambda: 0.2, lambda: 0, lambda: 0, lambda: False  # True
                    )
                )
            ).withTimeout(5)
        )
        .andThen(drivetrain.stop())
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    )
