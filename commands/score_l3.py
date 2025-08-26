from commands2 import SequentialCommandGroup, WaitCommand

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist

from commands.elevator_l3 import ElevatorL3


def score_l3_on_true(elevator: Elevator, wrist: Wrist) -> SequentialCommandGroup:
    return SequentialCommandGroup(
        wrist.angle_zero(), ElevatorL3(elevator), wrist.angle_score_l3()
    )
