from math import pi
from commands2 import Command
from wpimath.geometry import Rotation2d

from subsystems.wrist import Wrist


class WristAngleSlow(Command):
    def __init__(self, wrist: Wrist, angle: Rotation2d):
        super().__init__()
        self.wrist = wrist
        self.angle = angle

        self.addRequirements(self.wrist)

    def execute(self):
        if self.wrist.get_angle().radians() < self.angle.radians():
            # not sure this is the best way, robot architecture is currently flawed, but not going to fix for this
            self.wrist.manual_control(lambda: 0.1).execute()
        if self.wrist.get_angle().radians() > self.angle.radians():
            # not sure this is the best way, robot architecture is currently flawed, but not going to fix for this
            self.wrist.manual_control(lambda: -0.1).execute()

    def isFinished(self) -> bool:
        # not sure what tolerance is right
        return abs(self.wrist.get_angle().radians() - self.angle.radians()) < pi / 8

    def end(self, interrupted: bool):
        self.wrist._stop()
