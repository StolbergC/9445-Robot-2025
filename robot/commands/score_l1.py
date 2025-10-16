from commands2 import SequentialCommandGroup, WaitCommand

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist

from commands.elevator_l1 import ElevatorL1
from commands.wrist_l1 import WristL1


def score_l1_on_true(elevator: Elevator, wrist: Wrist) -> SequentialCommandGroup:
    return WristL1(wrist).andThen(ElevatorL1(elevator))
