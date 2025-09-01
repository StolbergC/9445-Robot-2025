from commands2 import Command, SequentialCommandGroup, ParallelCommandGroup

from subsystems.claw import Claw
from subsystems.wrist import Wrist
from subsystems.elevator import Elevator

from commands.claw_neutral import ClawNeutral
from commands.wrist_angle_zero import WristZero
from commands.wrist_intake import WristIntake
from commands.elevator_bottom import ElevatorBottom


def get_stow(elevator: Elevator, wrist: Wrist, claw: Claw) -> Command:
    return SequentialCommandGroup(
        ParallelCommandGroup(ClawNeutral(claw), WristZero(wrist)),
        ElevatorBottom(elevator),
        WristIntake(wrist),
    )
