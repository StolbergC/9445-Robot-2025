from commands.fingers_velocity import FingersVelocity


class FingersStop(FingersVelocity):
    velocity_rpm: float = 0

    def isFinished(self) -> bool:
        return super().isFinished() or abs(self.fingers.get_velocity().degrees()) < 2.5
