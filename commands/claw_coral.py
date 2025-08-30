from wpimath.units import meters

from commands.claw_distance import ClawDistance


class ClawCoral(ClawDistance):
    distance: meters = -0.1

    def isFinished(self) -> bool:
        return (
            self.claw.at_setpoint()
            and abs(self.distance - self.claw.get_setpoint()) < 0.01  # float == check
        ) or self.claw.at_center()
