from commands2 import SequentialCommandGroup, Command

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist
from subsystems.claw import Claw


def smack_algae_on_true(
    elevator: Elevator, wrist: Wrist, claw: Claw, low: bool = True
) -> SequentialCommandGroup:
    return (
        wrist.angle_zero()
        .andThen(claw.coral())
        .andThen((elevator.algae_intake_low() if low else elevator.algae_intake_high()))
        .andThen(claw.stop())
    )


def smack_alage_on_false(elevator: Elevator, wrist: Wrist) -> Command:
    return wrist.angle_intake()
