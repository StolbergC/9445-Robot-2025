from commands2 import Command, InstantCommand, WaitCommand, WrapperCommand
import commands2
from wpilib import RobotBase

from subsystems.drivetrain import Drivetrain
from subsystems.claw import Claw
from subsystems.elevator import Elevator
from subsystems.wrist import Wrist
from subsystems.fingers import Fingers

from auto import positions

from commands import score_l1, score, intake, score_processor

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
            drivetrain.reset_pose(positions.red_start_line_center)
            .andThen(
                drivetrain.set_speed_command(
                    feetToMeters(25), Rotation2d.fromDegrees(480)
                )
                if RobotBase.isReal()
                else commands2.cmd.none()
            )
            .alongWith(
                wrist.angle_intake().andThen(claw.home_outside()).andThen(claw.coral())
            )
            .andThen(wrist.angle_zero())
        )
        # score preload on l1
        .andThen(
            drivetrain.drive_position(positions.red_reef_h).alongWith(
                score_l1.score_l1_on_true(elevator, wrist)
            )
        )
        .andThen(score.score_coral(fingers, 1))
        # grab gh algae
        .andThen(drivetrain.drive_position(positions.red_algae_gh_far))
        .andThen(
            intake.intake_algae_low(elevator, wrist, claw).alongWith(
                drivetrain.drive_position(positions.red_algae_gh)
            )
        )
        # score
        .andThen(drivetrain.drive_position(positions.red_algae_gh_far))
        .andThen(
            drivetrain.drive_position(positions.red_processor).alongWith(
                score_processor.score_processor_on_true(elevator, wrist)
            )
        )
        .andThen(score.score_alage(fingers, 1))
        .andThen(
            drivetrain.drive_position(positions.red_algae_ef).alongWith(
                intake.intake_algae_high(elevator, wrist, claw)
            )
        )
        .andThen(intake.intake_algae_high(elevator, wrist, claw))
        .andThen(drivetrain.drive_position(positions.red_algae_ef_far))
        .andThen(
            drivetrain.drive_position(positions.red_processor).alongWith(
                score_processor.score_processor_on_true(elevator, wrist)
            )
        )
        .andThen(score.score_alage(fingers, 2))
        .andThen(
            drivetrain.set_speed_command(start_max_vel_mps, start_max_angular_vel)
            if RobotBase.isReal()
            else commands2.cmd.none()
        )
        .withName("Red Center Two Algae")
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    )
