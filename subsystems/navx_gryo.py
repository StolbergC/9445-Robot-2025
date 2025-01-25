from subsystems.gyro_base import GyroBase

from navx import AHRS

from wpilib import SerialPort

from wpimath.geometry import Rotation2d

from typing import Self


class NavX(GyroBase):
    def __init__(
        self,
        serial_type: AHRS.NavXComType,
    ):
        self.hardware = AHRS(serial_type)

    def fromMXP():
        return NavX(AHRS.NavXComType.kMXP_UART)

    def fromUSB(port: int) -> Self:
        if port == 1:
            return NavX(AHRS.NavXComType.kUSB1)
        elif port == 2:
            return NavX(AHRS.NavXComType.kUSB2)
        else:
            raise IndexError(
                "The roborio only has 2 usb ports. Your port can only be 1 or 2 [0]"
            )

    def get_angle(self):
        return self.hardware.getRotation2d() + Rotation2d.fromDegrees(90)

    def reset(self, new_angle: Rotation2d = Rotation2d.fromDegrees(0)) -> None:
        self.hardware.reset()
        self.hardware.setAngleAdjustment(new_angle.degrees())
