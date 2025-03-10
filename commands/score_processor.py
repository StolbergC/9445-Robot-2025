from commands2 import SequentialCommandGroup, WaitCommand

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist


def score_processor_on_true(elevator: Elevator, wrist: Wrist) -> SequentialCommandGroup:
    return (
        wrist.angle_zero()
        .andThen(elevator.set_setpoint_processor())
        .andThen(WaitCommand(0.1).until(elevator.close))
    )
