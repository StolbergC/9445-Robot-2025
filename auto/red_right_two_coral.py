from commands2 import Command, InstantCommand, WaitCommand, WrapperCommand
import commands2
from wpilib import RobotBase

from commands import intake, score, score_l2
from subsystems.drivetrain import Drivetrain
from subsystems.elevator import Elevator
from subsystems.wrist import Wrist
from subsystems.claw import Claw
from subsystems.fingers import Fingers

from auto import positions

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
            drivetrain.reset_pose(positions.red_start_line_right)
            .alongWith(
                drivetrain.set_speed_command(
                    feetToMeters(25), Rotation2d.fromDegrees(480)
                )
                if RobotBase.isReal()
                else commands2.cmd.none()
            )
            .alongWith(wrist.angle_intake().andThen(claw.coral()))
            .andThen(wrist.angle_zero())
            .andThen(elevator.home())
        )
        .andThen(
            score_l2.score_l2_on_true(elevator, wrist).alongWith(
                drivetrain.drive_position(positions.red_reef_f)
            )
        )
        .andThen(score.score_coral(claw, fingers, 1))
        .andThen(
            drivetrain.drive_position(positions.red_coral_intake_right_right).alongWith(
                intake.intake_coral(elevator, wrist, claw, fingers, 1)
            )
        )
        .andThen(
            intake.intake_coral(elevator, wrist, claw, fingers, 1)
        )  # for if the drive beats the intake
        .andThen(
            drivetrain.drive_position(positions.red_reef_e).alongWith(
                score_l2.score_l2_on_true(elevator, wrist)
            )
        )
        .andThen(score.score_coral(claw, fingers, 2))
        .andThen(  # maybe happens, TODO: Test, if unable to reach, get rid of this
            drivetrain.drive_position(positions.red_coral_intake_right_right).alongWith(
                intake.intake_coral(elevator, wrist, claw, fingers, 2)
            )
        )
        .andThen(intake.intake_coral(elevator, wrist, claw, fingers, 1))
        .andThen(claw.coral())
        .withName("Red Left Four Coral")
        .withInterruptBehavior(
            Command.InterruptionBehavior.kCancelIncoming
        )  # I do not know why this is needed, but auto does not run in the autonomousInit without it
    )
