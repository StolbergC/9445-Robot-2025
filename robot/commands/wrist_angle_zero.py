from commands.wrist_angle import WristAngle
from wpimath.geometry import Rotation2d


class WristZero(WristAngle):
    setpoint: Rotation2d = Rotation2d.fromDegrees(0)
