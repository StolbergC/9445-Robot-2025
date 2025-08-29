from commands2 import (
    Command,
    ParallelCommandGroup,
    SequentialCommandGroup,
    WaitCommand,
    WrapperCommand,
)

from commands.elevator_bottom import ElevatorBottom
from commands.wrist_intake import WristIntake
from commands.wrist_angle_zero import WristZero

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist
from subsystems.claw import Claw
from subsystems.fingers import Fingers

from commands.elevator_intake import ElevatorIntake


def intake_coral(
    elevator: Elevator,
    wrist: Wrist,
    claw: Claw,
) -> WrapperCommand:
    return (
        (
            SequentialCommandGroup(
                WristZero(wrist),
                ElevatorIntake(elevator),
                claw.cage(),
                WristIntake(wrist),
            )
        )
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        .withName("Intake Coral")
    )
