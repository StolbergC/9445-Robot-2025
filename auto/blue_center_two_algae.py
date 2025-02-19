from commands2 import Command, InstantCommand, WaitCommand, WrapperCommand
import commands2
from wpilib import RobotBase
from subsystems.drivetrain import Drivetrain

from auto import positions

from wpimath.geometry import Rotation2d
from wpimath.units import feetToMeters


def get_auto(drivetrain: Drivetrain) -> WrapperCommand:
    start_max_vel_mps = drivetrain.max_velocity_mps
    start_max_angular_vel = drivetrain.max_angular_velocity
    return (
        (
            drivetrain.reset_pose(positions.blue_reef_center).alongWith(
                drivetrain.set_speed_command(
                    feetToMeters(25), Rotation2d.fromDegrees(480)
                )
                if RobotBase.isReal()
                else commands2.cmd.none()
            )
        )
        .andThen(WaitCommand(0.1))
        .andThen(drivetrain.drive_position(positions.blue_reef_h))
        .andThen(WaitCommand(0.25))  # score
        .andThen(drivetrain.drive_position(positions.blue_algae_gh))
        .andThen(WaitCommand(0.25))  # grab algae
        .andThen(drivetrain.drive_position(positions.blue_processor))
        .andThen(WaitCommand(0.25))  # score
        .andThen(drivetrain.drive_position(positions.blue_algae_ef))
        .andThen(WaitCommand(0.25))  # grab algae
        .andThen(drivetrain.drive_position(positions.blue_processor))
        .andThen(
            drivetrain.set_speed_command(start_max_vel_mps, start_max_angular_vel)
            if RobotBase.isReal()
            else commands2.cmd.none()
        )
        .withName("Blue Center Two Algae")
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    )
