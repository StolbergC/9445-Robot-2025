from commands2 import SequentialCommandGroup

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist


def score_l3_on_true(elevator: Elevator, wrist: Wrist) -> SequentialCommandGroup:
    return (
        wrist.angle_zero().andThen(elevator.command_l3()).andThen(wrist.angle_score())
    )
