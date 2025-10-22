from commands2 import Command, SequentialCommandGroup, WaitCommand

from subsystems.wrist import Wrist
from subsystems.elevator import Elevator

from commands.wrist_l3 import WristL3
from commands.wrist_intake import WristIntake
from commands.elevator_bottom import ElevatorBottom


def get_stow(elevator: Elevator, wrist: Wrist) -> Command:
    return SequentialCommandGroup(
        WristL3(wrist),
        WaitCommand(0.25),
        ElevatorBottom(elevator),
        WaitCommand(1.5),
        WristIntake(wrist),
    )
