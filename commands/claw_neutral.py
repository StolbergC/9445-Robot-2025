from wpimath.units import meters, inchesToMeters

from commands.claw_distance import ClawDistance


class ClawNeutral(ClawDistance):
    distance: meters = inchesToMeters(8)

    def isFinished(self) -> bool:
        return (
            self.claw.at_setpoint()
            and abs(self.distance - self.claw.get_setpoint()) < 0.01  # float == check
        ) or self.claw.at_center()
