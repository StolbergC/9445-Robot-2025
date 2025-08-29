from commands.wrist_angle import WristAngle
from wpimath.geometry import Rotation2d


class WristL1(WristAngle):
    setpoint: Rotation2d = Rotation2d.fromDegrees(0)
