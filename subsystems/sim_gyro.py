from wpimath.geometry import Rotation2d


class SimGyro:
    def __init__(self): ...

    def get_angle(self) -> Rotation2d: 
        return Rotation2d.fromDegrees(0)

    def reset(self, new_angle: Rotation2d = Rotation2d.fromDegrees(0)) -> None: ...
