from commands2 import Command

from subsystems.wrist import Wrist

from wpimath.geometry import Rotation2d


class WristAngle(Command):
    setpoint: Rotation2d

    def __init__(self, wrist: Wrist):
        self.wrist = wrist
        self.addRequirements(wrist)
        self.setName(f"Wrist Angle {self.setpoint.degrees()}")

    def initialize(self) -> None:
        self.wrist.set_setpoint(self.setpoint)

    def isFinished(self) -> bool:
        return (
            self.wrist.at_setpoint()
            and abs(self.wrist.get_setpoint().degrees() - self.setpoint.degrees())
            < 0.01  # this is == but for float precision
        )
