from commands2 import SequentialCommandGroup

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist


def score_processor_on_true(elevator: Elevator, wrist: Wrist) -> SequentialCommandGroup:
    return wrist.angle_zero().andThen(elevator.command_processor())
