from wpimath.geometry import Rotation2d


class SimGyro:
    def __init__(self):
        self.angle = Rotation2d(0)

    def get_angle(self) -> Rotation2d:
        return self.angle

    def reset(self, new_angle: Rotation2d = Rotation2d.fromDegrees(0)) -> None:
        self.angle = new_angle
