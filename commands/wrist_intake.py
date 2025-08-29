from commands.wrist_angle import WristAngle
from wpimath.geometry import Rotation2d


class WristIntake(WristAngle):
    setpoint: Rotation2d = Rotation2d.fromDegrees(60)
