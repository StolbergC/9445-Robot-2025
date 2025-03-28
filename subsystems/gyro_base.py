from wpimath.geometry import Rotation2d


class GyroBase:
    def __init__(self): ...

    def get_angle(self) -> Rotation2d: ...

    def reset(self, new_angle: Rotation2d = Rotation2d.fromDegrees(0)) -> None: ...
