from auto import positions
from subsystems.drivetrain import Drivetrain

from wpimath.geometry import Pose2d, Rotation2d
from commands2 import Command, DeferredCommand, RepeatCommand

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist
from subsystems.claw import Claw
from subsystems.fingers import Fingers

from commands import score_l1


def get_auto(
    drivetrain: Drivetrain,
    elevator: Elevator,
    wrist: Wrist,
    claw: Claw,
    fingers: Fingers,
) -> Command:
    return (
        drivetrain.reset_pose(positions.blue_start_line_center)
        .andThen(
            RepeatCommand(
                DeferredCommand(
                    lambda: drivetrain.drive_joystick(
                        lambda: 0.3, lambda: 0, lambda: 0, lambda: False  # True
                    )
                )
            )
            .withTimeout(10)
            .alongWith(score_l1.score_l1_on_true(elevator, wrist))
        )
        .andThen(drivetrain.stop())
        .andThen(fingers.score().withTimeout(5))
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    )
