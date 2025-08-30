from wpimath.units import meters

from commands.claw_distance import ClawDistance


class ClawAlgae(ClawDistance):
    distance: meters = 500  # at_outside covers the real distance required

    def isFinished(self) -> bool:
        return (
            self.claw.at_setpoint()
            and abs(self.distance - self.claw.get_setpoint()) < 0.01  # float == check
        ) or self.claw.at_outside()
