from commands2 import ParallelCommandGroup, SequentialCommandGroup

from subsystems.claw import Claw
from subsystems.fingers import Fingers


def score(claw: Claw, fingers: Fingers) -> ParallelCommandGroup:
    return fingers.score().alongWith(claw.cage())  # cage is just further out
