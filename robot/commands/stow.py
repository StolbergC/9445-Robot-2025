from commands2 import Command, SequentialCommandGroup, ParallelCommandGroup

from subsystems.wrist import Wrist
from subsystems.elevator import Elevator

from commands.wrist_angle_zero import WristZero
from commands.wrist_intake import WristIntake
from commands.elevator_bottom import ElevatorBottom


def get_stow(elevator: Elevator, wrist: Wrist) -> Command:
    return SequentialCommandGroup(
        WristZero(wrist),
        ElevatorBottom(elevator),
        WristIntake(wrist),
    )
