from turtle import position
from commands2 import Command, InstantCommand, WaitCommand, WrapperCommand
import commands2
from wpilib import RobotBase

from subsystems.drivetrain import Drivetrain
from subsystems.claw import Claw
from subsystems.elevator import Elevator
from subsystems.wrist import Wrist
from subsystems.fingers import Fingers

from auto import positions

from commands import score_l1, score, intake, score_processor, score_l3

from wpimath.geometry import Rotation2d
from wpimath.units import feetToMeters


def get_auto(
    drivetrain: Drivetrain,
    elevator: Elevator,
    wrist: Wrist,
    claw: Claw,
    fingers: Fingers,
) -> WrapperCommand:
    start_max_vel_mps = drivetrain.max_velocity_mps
    start_max_angular_vel = drivetrain.max_angular_velocity
    return (
        # setup
        (
            drivetrain.reset_pose(positions.blue_start_line_center)
            .andThen(
                drivetrain.set_speed_command(
                    feetToMeters(12), Rotation2d.fromDegrees(180)
                )
                if RobotBase.isReal()
                else commands2.cmd.none()
            )
            .alongWith(wrist.angle_intake().andThen(claw.coral()))
        )
        .andThen(
            score_l1.score_l1_on_true(elevator, wrist).alongWith(
                drivetrain.drive_position(positions.blue_algae_gh)
            )
        )
        .andThen(elevator.command_l3())
        .andThen(drivetrain.drive_position(positions.blue_algae_gh_far))
        .andThen(
            drivetrain.drive_position(positions.blue_reef_g).alongWith(
                score_l3.score_l3_on_true(elevator, wrist)
            )
        )
        .andThen(score.score_coral(fingers, 2))
        .andThen(fingers.stop())
        .andThen(
            drivetrain.set_speed_command(start_max_vel_mps, start_max_angular_vel)
            if RobotBase.isReal()
            else commands2.cmd.none()
        )
        .withName("Blue Center")
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    )
