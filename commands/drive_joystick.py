from math import pi
from typing import Callable

from commands2 import Command
from wpilib import RobotBase, Timer

from subsystems.drivetrain import Drivetrain

from wpimath.geometry import Rotation2d, Translation2d
from wpimath.controller import PIDController


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
        self.nettable = self.drivetrain.nettable.getSubTable("00DriveJoystick")
        self.get_x = get_x
        self.get_y = get_y
        self.get_omega = get_omega
        self.get_field_oriented = get_field_oriented
        self.addRequirements(self.drivetrain)
        self.angle_hold: Rotation2d = self.drivetrain.get_angle()
        self.omega_compensator = PIDController(0.5, 0, 0)
        self.omega_compensator.enableContinuousInput(-pi, pi)
        self.prev_used_omega = False
        self.spin_compensation_timer = Timer()
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

        # deadband included here
        if omega == 0:
            if self.prev_used_omega:
                self.angle_hold = self.drivetrain.get_angle()
                self.spin_compensation_timer.start()
            # allow the modules to settle to prevent overshoot
            if self.spin_compensation_timer.hasElapsed(0.5):
                omega -= (
                    f := self.omega_compensator.calculate(
                        self.drivetrain.get_angle().radians(), self.angle_hold.radians()
                    )
                )
                self.nettable.putBoolean("Compensating", True)
                self.nettable.putNumber("Compensation Factor", f)
                self.nettable.putNumber("Angle Hold (deg)", self.angle_hold.degrees())
                self.nettable.putNumber("Angle Hold (rad)", self.angle_hold.radians())
                self.prev_used_omega = False

        else:
            self.spin_compensation_timer.reset()
            self.prev_used_omega = True
            self.nettable.putBoolean("Compensating", False)

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
