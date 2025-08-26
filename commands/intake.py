from commands2 import (
    Command,
    ParallelCommandGroup,
    SequentialCommandGroup,
    WaitCommand,
    WrapperCommand,
)

from commands.elevator_bottom import ElevatorBottom
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
                wrist.angle_zero(),
                ElevatorIntake(elevator),
                claw.cage(),
                wrist.angle_intake(),
            )
        )
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        .withName("Intake Coral")
    )
