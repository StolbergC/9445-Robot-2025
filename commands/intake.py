from commands2 import ParallelCommandGroup, SequentialCommandGroup

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist
from subsystems.claw import Claw
from subsystems.fingers import Fingers

from commands.score import score_alage, score_coral


def _upper_intake_coral(claw: Claw, fingers: Fingers) -> ParallelCommandGroup:
    return claw.coral().alongWith(fingers.intake())


def intake_coral(
    elevator: Elevator,
    wrist: Wrist,
    claw: Claw,
    fingers: Fingers,
    intake_timeout: float = 0,
) -> SequentialCommandGroup:
    if intake_timeout > 0:
        return (
            wrist.angle_zero()
            .andThen(elevator.command_intake())
            .andThen(
                (
                    wrist.angle_intake().alongWith(_upper_intake_coral(claw, fingers))
                ).withTimeout(intake_timeout)
            )
        )
    return (
        wrist.angle_zero()
        .andThen(elevator.command_intake())
        .andThen(wrist.angle_intake().alongWith(_upper_intake_coral(claw, fingers)))
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
        return intake_algae_low(elevator, wrist, claw, fingers)
    elif level == 3:
        return intake_algae_high(elevator, wrist, claw, fingers)
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
