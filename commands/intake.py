from commands2 import (
    Command,
    ParallelCommandGroup,
    SequentialCommandGroup,
    WrapperCommand,
)

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist
from subsystems.claw import Claw
from subsystems.fingers import Fingers


def _upper_intake_coral(claw: Claw, fingers: Fingers) -> ParallelCommandGroup:
    return claw.cage().alongWith(fingers.intake())


def intake_coral(
    elevator: Elevator,
    wrist: Wrist,
    claw: Claw,
    fingers: Fingers,
    intake_timeout: float = 0,
) -> SequentialCommandGroup | WrapperCommand:
    if intake_timeout > 0:
        return (
            wrist.angle_zero()
            .andThen(elevator.command_intake().alongWith(claw.cage()))
            .andThen(wrist.angle_intake())
            .andThen(fingers.intake().withTimeout(intake_timeout))
        )
    return (
        wrist.angle_zero()
        .andThen(elevator.command_intake().alongWith(claw.cage()))
        .andThen(wrist.angle_intake())
        .andThen(fingers.intake())
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
    )


def intake_algae(
    elevator: Elevator,
    wrist: Wrist,
    claw: Claw,
    fingers: Fingers,
    level: int,
    intake_timeout: float = 0,
) -> SequentialCommandGroup:
    if level == 2:
        return intake_algae_low(elevator, wrist, claw, fingers, intake_timeout)
    elif level == 3:
        return intake_algae_high(elevator, wrist, claw, fingers, intake_timeout)
    else:
        raise ValueError("The only levels for algae are 2 and 3")


def intake_algae_low(
    elevator: Elevator,
    wrist: Wrist,
    claw: Claw,
    fingers: Fingers,
    intake_timeout: float = 0,
) -> SequentialCommandGroup:
    if intake_timeout > 0:
        return (
            wrist.angle_zero()
            .andThen(elevator.algae_intake_low())
            .andThen(
                (claw.algae().alongWith(fingers.intake(False))).withTimeout(
                    intake_timeout
                )
            )
        )
    return (
        wrist.angle_zero()
        .andThen(elevator.algae_intake_low())
        .andThen(claw.algae().alongWith(fingers.intake(False)))
    )


def intake_algae_high(
    elevator: Elevator,
    wrist: Wrist,
    claw: Claw,
    fingers: Fingers,
    intake_timeout: float = 0,
) -> SequentialCommandGroup:
    if intake_timeout > 0:
        return (
            wrist.angle_zero()
            .andThen(elevator.algae_intake_high())
            .andThen(
                (claw.algae().alongWith(fingers.intake(False))).withTimeout(
                    intake_timeout
                )
            )
        )
    return (
        wrist.angle_zero()
        .andThen(elevator.algae_intake_high())
        .andThen(claw.algae().alongWith(fingers.intake(False)))
    )
