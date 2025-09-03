from commands2 import SequentialCommandGroup, WaitCommand

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist

from commands.elevator_l2 import ElevatorL2
from commands.wrist_l2 import WristL2
from commands.wrist_angle_zero import WristZero


def score_l2_on_true(elevator: Elevator, wrist: Wrist) -> SequentialCommandGroup:
    return SequentialCommandGroup(
        WristZero(wrist).onlyIf(
            lambda: wrist.get_angle().degrees() > 40
            or wrist.get_angle().degrees() < -40
        ),
        ElevatorL2(elevator),
        WristL2(wrist),
    )
