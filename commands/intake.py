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

from commands.elevator_intake import ElevatorIntake
from commands.claw_neutral import ClawNeutral
from commands.claw_coral import ClawCoral


def intake_coral(
    elevator: Elevator,
    wrist: Wrist,
    claw: Claw,
) -> WrapperCommand:
    return (
        (
            SequentialCommandGroup(
                WristZero(wrist).onlyIf(lambda: wrist.get_angle().degrees() > 60),
                ParallelCommandGroup(
                    ElevatorIntake(elevator),
                    ClawNeutral(claw),
                ),
                WristIntake(wrist),
            )
        )
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        .withName("Intake Coral")
    )


def pinch_coral(claw: Claw) -> Command:
    return ClawCoral(claw)
