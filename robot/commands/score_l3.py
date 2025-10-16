from commands2 import SequentialCommandGroup, WaitCommand

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist

from commands.elevator_l3 import ElevatorL3
from commands.wrist_l3 import WristL3
from commands.wrist_angle_zero import WristZero


def score_l3_on_true(elevator: Elevator, wrist: Wrist) -> SequentialCommandGroup:
    return SequentialCommandGroup(
        WristZero(wrist).onlyIf(
            lambda: wrist.get_angle().degrees() > 40
            or wrist.get_angle().degrees() < -40
        ),
        ElevatorL3(elevator),
        WristL3(wrist),
    )
