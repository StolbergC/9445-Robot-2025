from typing import Callable
from commands2 import Command

from subsystems.drivetrain import CommandSwerveDrivetrain

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from pathplannerlib.auto import PathConstraints


from wpimath.geometry import Rotation2d, Transform2d, Pose2d
from wpimath.units import inchesToMeters

from commands.autoalign_reef import autoalign_reef_offset


field = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)
BLUE = [
    Pose2d(
        pose.X(),
        pose.Y(),
        pose.rotation().toRotation2d().rotateBy(Rotation2d.fromDegrees(180)),
    )
    for tag_id in [12, 13]
    if (pose := field.getTagPose(tag_id)) is not None
]
RED = [
    Pose2d(
        pose.X(),
        pose.Y(),
        pose.rotation().toRotation2d().rotateBy(Rotation2d.fromDegrees(180)),
    )
    for tag_id in [1, 2]
    if (pose := field.getTagPose(tag_id)) is not None
]


def autoalign_intake(
    drivetrain: CommandSwerveDrivetrain,
    constraints: PathConstraints,
    offset: Transform2d = Transform2d(
        inchesToMeters(-20), inchesToMeters(0), Rotation2d(0)
    ),
    wait_until: Callable[[], bool] = lambda: True,
) -> Command:
    return autoalign_reef_offset(
        drivetrain, constraints, offset, wait_until=wait_until, blue=BLUE, red=RED
    )
