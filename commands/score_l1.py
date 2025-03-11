from commands2 import SequentialCommandGroup, WaitCommand

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist


def score_l1_on_true(elevator: Elevator, wrist: Wrist) -> SequentialCommandGroup:
    return wrist.angle_zero().andThen(
        elevator.command_l1()
    )  # we are just at 0 degrees for l1
