from commands2 import SequentialCommandGroup, WaitCommand

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist

from commands.elevator_l2 import ElevatorL2


def score_l2_on_true(elevator: Elevator, wrist: Wrist) -> SequentialCommandGroup:
    return SequentialCommandGroup(
        wrist.angle_zero(),
        ElevatorL2(elevator),
        wrist.angle_score(),
    )
