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
from subsystems.fingers import Fingers

from commands.elevator_intake import ElevatorIntake
from commands.fingers_intake import FingersIntake


def intake_coral(elevator: Elevator, wrist: Wrist, fingers: Fingers) -> WrapperCommand:
    return (
        (
            SequentialCommandGroup(
                WristZero(wrist).onlyIf(
                    lambda: wrist.get_angle().degrees() > 20
                    or wrist.get_angle().degrees() < -30
                ),
                ElevatorIntake(elevator),
                WristIntake(wrist),
                FingersIntake(fingers),
            )
        )
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        .withName("Intake Coral")
    )
