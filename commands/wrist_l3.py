from commands.wrist_angle import WristAngle
from wpimath.geometry import Rotation2d


class WristL3(WristAngle):
    setpoint: Rotation2d = Rotation2d.fromDegrees(-10)
