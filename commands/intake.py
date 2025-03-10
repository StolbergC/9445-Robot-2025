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
) -> WrapperCommand:
    return (
        wrist.angle_zero()
        .andThen(elevator.setpoint_intake().alongWith(claw.cage()))
        .andThen(wrist.angle_intake())
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        .withName("Intake Coral")
    )


def intake_algae(
    elevator: Elevator,
    wrist: Wrist,
    claw: Claw,
    fingers: Fingers,
    level: int,
    intake_timeout: float = 0,
) -> SequentialCommandGroup:
    if level == 1:
        return intake_algae_ground(elevator, wrist, claw)
    elif level == 2:
        return intake_algae_low(elevator, wrist, claw)
    elif level == 3:
        return intake_algae_high(elevator, wrist, claw)
    else:
        raise ValueError("The only levels for algae are 1, 2, and 3")


def intake_algae_ground(
    elevator: Elevator,
    wrist: Wrist,
    claw: Claw,
) -> SequentialCommandGroup:
    return (wrist.angle_score()).andThen(
        elevator.command_bottom().alongWith(claw.algae_outside())
    )


def intake_algae_low(
    elevator: Elevator,
    wrist: Wrist,
    claw: Claw,
) -> SequentialCommandGroup:
    return wrist.angle_zero().andThen(
        elevator.setpoint_algae_intake_low().alongWith(claw.algae_outside())
    )


def intake_algae_high(
    elevator: Elevator,
    wrist: Wrist,
    claw: Claw,
) -> SequentialCommandGroup:
    return wrist.angle_zero().andThen(
        elevator.setpoint_algae_intake_high()
        .andThen(wrist.angle_intake())
        .alongWith(claw.algae_outside())
    )
