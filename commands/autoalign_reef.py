from commands2 import Command, DeferredCommand, SelectCommand, SequentialCommandGroup
import commands2

from subsystems.drivetrain import CommandSwerveDrivetrain

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from pathplannerlib.auto import AutoBuilder, PathConstraints

from wpimath.geometry import Rotation2d, Transform2d, Pose2d
from wpimath.units import inchesToMeters

from commands.pid_align import PIDAlign


field = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)
BLUE = [
    Pose2d(
        pose.X(),
        pose.Y(),
        pose.rotation().toRotation2d().rotateBy(Rotation2d.fromDegrees(180)),
    )
    for tag_id in [17, 18, 19, 20, 21, 22]
    if (pose := field.getTagPose(tag_id)) is not None
]
RED = [
    Pose2d(
        pose.X(),
        pose.Y(),
        pose.rotation().toRotation2d().rotateBy(Rotation2d.fromDegrees(180)),
    )
    for tag_id in [6, 7, 8, 9, 10, 11]
    if (pose := field.getTagPose(tag_id)) is not None
]


def _get_near_pose(drivetrain: CommandSwerveDrivetrain) -> Pose2d:
    return drivetrain.get_state().pose.nearest(
        BLUE if drivetrain.get_operator_forward_direction() == Rotation2d(0) else RED
    )


def autoalign_reef_offset(
    drivetrain: CommandSwerveDrivetrain,
    constraints: PathConstraints,
    offset: Transform2d,
) -> Command:
    try:
        return DeferredCommand(
            lambda: SequentialCommandGroup(
                SelectCommand(
                    {
                        (
                            pose.X(),
                            pose.Y(),
                            pose.rotation().radians(),
                        ): AutoBuilder.pathfindToPose(pose + offset, constraints)
                        for pose in (
                            BLUE
                            if drivetrain.get_operator_forward_direction()
                            == Rotation2d(0)
                            else RED
                        )
                    },
                    lambda: (
                        (
                            (pose := _get_near_pose(drivetrain)).X(),
                            pose.Y(),
                            pose.rotation().radians(),
                        )
                    ),
                ),
                PIDAlign(
                    drivetrain, _get_near_pose(drivetrain) + offset
                ),  # fix any weird pathplanner problems,
            ).onlyIf(
                lambda: 0 < (p := drivetrain.get_state().pose).X()
                and p.X() < field.getFieldLength()
                and 0 < p.Y()
                and p.Y() < field.getFieldWidth()
            ),
            drivetrain,
        )
    except ZeroDivisionError:
        # this happens if the robot believes that it is inside of an obstacle
        return commands2.cmd.none()


def autoalign_reef_right(
    drivetrain: CommandSwerveDrivetrain,
    constraints: PathConstraints,
) -> Command:
    return autoalign_reef_offset(
        drivetrain,
        constraints,
        Transform2d(-inchesToMeters(24), -inchesToMeters(13 / 2), 0),
    )


def autoalign_reef_left(
    drivetrain: CommandSwerveDrivetrain,
    constraints: PathConstraints,
) -> Command:
    return autoalign_reef_offset(
        drivetrain,
        constraints,
        Transform2d(-inchesToMeters(24), inchesToMeters(13 / 2), 0),
    )
