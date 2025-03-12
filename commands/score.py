from commands2 import (
    ParallelCommandGroup,
    ParallelRaceGroup,
    SequentialCommandGroup,
    Command,
)
from wpimath.units import seconds

from subsystems.claw import Claw
from subsystems.fingers import Fingers


def score_coral(
    fingers: Fingers, timeout: seconds = 0
) -> ParallelCommandGroup | ParallelRaceGroup:
    if timeout > 0:
        return fingers.score().withTimeout(timeout)
    return fingers.score()  # cage is just further out


def score_alage(fingers: Fingers, timeout: seconds = 0) -> Command:
    if timeout > 0:
        return fingers.score().withTimeout(timeout)
    return fingers.score()
