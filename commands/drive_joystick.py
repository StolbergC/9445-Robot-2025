from typing import Callable

from commands2 import Command
from wpilib import RobotBase

from subsystems.drivetrain import Drivetrain

from wpimath.geometry import Rotation2d, Translation2d


class DriveJoystick(Command):
    fudge_factor: float = 8.5 if RobotBase.isReal() else 8.5

    def __init__(
        self,
        drivetrain: Drivetrain,
        get_x: Callable[[], float],
        get_y: Callable[[], float],
        get_omega: Callable[[], float],
        get_field_oriented: Callable[[], bool],
    ):
        self.drivetrain = drivetrain
        self.get_x = get_x
        self.get_y = get_y
        self.get_omega = get_omega
        self.get_field_oriented = get_field_oriented
        self.addRequirements(self.drivetrain)
        super().__init__()

    def initialize(self):
        return super().initialize()

    def execute(self):
        # https://vamfun.wordpress.com/2024/11/08/swerve-drive-lateral-drift-when-spinning-as-you-translate-cause-and-proposed-command-compensation-fix/
        # above provides insipration for below algorithm

        omega = self.get_omega()

        sigma = (
            self.fudge_factor * 0.02 * (omega * self.drivetrain.max_angular_speed) / 2
        )
        pre_input = Translation2d(self.get_x(), self.get_y())
        out_translation = pre_input.rotateBy(Rotation2d(sigma))
        x_out = out_translation.X()
        y_out = out_translation.Y()

        self.drivetrain.run_percent(
            x_out,
            y_out,
            omega,
            self.get_field_oriented(),
        )
        return super().execute()

    def end(self, interrupted):
        # self.drivetrain.stop()
        return super().end(interrupted)
