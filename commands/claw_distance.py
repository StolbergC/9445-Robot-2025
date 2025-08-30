from commands2 import Command
from wpimath.units import meters

from subsystems.claw import Claw


class ClawDistance(Command):
    distance: meters

    def __init__(self, claw: Claw):
        super().__init__()
        self.claw = claw
        self.addRequirements(self.claw)
        self.setName(f"ClawDistanceCommand {self.distance}")

    def initialize(self) -> None:
        self.claw.set_setpoint(self.distance)

    def isFinished(self) -> bool:
        return (
            self.claw.at_setpoint()
            and abs(self.distance - self.claw.get_setpoint()) < 0.01  # float == check
        )
