from commands2 import SequentialCommandGroup, WaitCommand

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist


def score_l2_on_true(elevator: Elevator, wrist: Wrist) -> SequentialCommandGroup:
    return (
        wrist.angle_zero().andThen(elevator.command_l2()).andThen(wrist.angle_score())
    )
