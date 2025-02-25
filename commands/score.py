from commands2 import ParallelCommandGroup, SequentialCommandGroup, Command

from subsystems.claw import Claw
from subsystems.fingers import Fingers


def score_coral(claw: Claw, fingers: Fingers) -> ParallelCommandGroup:
    return fingers.score().alongWith(claw.cage())  # cage is just further out


def score_alage(fingers: Fingers) -> Command:
    return fingers.score()
