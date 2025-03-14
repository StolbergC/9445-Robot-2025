from commands2 import SequentialCommandGroup, Command

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist


def smack_algae_on_true(
    elevator: Elevator, wrist: Wrist, low: bool = True
) -> SequentialCommandGroup:
    return wrist.angle_zero().andThen(
        (elevator.algae_intake_low() if low else elevator.algae_intake_high())
    )


def smack_alage_on_false(elevator: Elevator, wrist: Wrist) -> Command:
    return wrist.angle_intake()
