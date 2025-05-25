from typing import Callable

from commands2 import Command

from subsystems.drivetrain import Drivetrain


class DriveJoystick(Command):
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

    def execute(self):
        self.drivetrain.run_percent(
            self.get_x(), self.get_y(), self.get_omega(), self.get_field_oriented()
        )
        return super().execute()

    def end(self, interrupted):
        # self.drivetrain.stop()
        return super().end(interrupted)
