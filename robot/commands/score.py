from commands2 import Command, SequentialCommandGroup
from wpimath.units import seconds

from subsystems.fingers import Fingers
from commands.fingers_score import FingersScore
from commands.fingers_stop import FingersStop


def score_coral(fingers: Fingers, timeout: seconds = 0) -> Command:
    if timeout > 0:
        return SequentialCommandGroup(
            FingersScore(fingers, timeout), FingersStop(fingers)
        )
    return FingersScore(fingers)


"""
def score_alage(fingers: Fingers, timeout: seconds = 0) -> Command:
    if timeout > 0:
        return fingers.score().withTimeout(timeout)
    return fingers.score()
"""
