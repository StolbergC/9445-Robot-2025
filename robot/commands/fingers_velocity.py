from commands2 import Command

from commands2.command import InterruptionBehavior
from wpilib import Timer
from wpimath.geometry import Rotation2d
from wpimath.units import seconds

from subsystems.fingers import Fingers


class FingersVelocity(Command):
    velocity_rpm: float
    timeout: seconds | None = None
    timer: Timer | None = None

    def __init__(self, fingers: Fingers, timeout: seconds | None = None):
        self.fingers = fingers

        if timeout:
            self.timeout = timeout

        self.addRequirements(fingers)
        self.setName(f"Fingers {self.velocity_rpm} RPM")

        if self.timeout is not None:
            self.timer = Timer()

    def initialize(self) -> None:
        self.fingers.set_setpoint(Rotation2d.fromDegrees(self.velocity_rpm * 360))
        if self.timer:
            self.timer.restart()

    def isFinished(self) -> bool:
        return (
            self.timer is not None
            and self.timeout is not None
            and self.timer.hasElapsed(self.timeout)
        )

    def getInterruptionBehavior(self) -> InterruptionBehavior:
        return Command.InterruptionBehavior.kCancelSelf
