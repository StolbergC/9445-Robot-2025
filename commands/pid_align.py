from math import pi
from commands2 import Command

from subsystems.drivetrain import CommandSwerveDrivetrain

from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ProfiledPIDController

from phoenix6 import swerve


class PIDAlign(Command):
    # everything happens in meters and radians
    translation_kP: float = 1
    translation_kI: float = 0
    translation_kD: float = 0

    rotation_kP: float = 1
    rotation_kI: float = 0
    rotation_kD: float = 0

    translation_profile: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(
        3, 9
    )
    rotation_profile: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(3, 6)

    # set in initialize if on red
    invert = False

    def __init__(self, drivetrain: CommandSwerveDrivetrain, target_pose: Pose2d):
        super().__init__()
        self.drivetrain = drivetrain
        self.target_pose = target_pose
        self._drive = (
            swerve.requests.FieldCentric()
            # .with_deadband(0)  # deadband is handled in get_velocity_x/y
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )  # Use open-loop control for drive motors

        self.tX_pid = ProfiledPIDController(
            self.translation_kP,
            self.translation_kI,
            self.translation_kD,
            self.translation_profile,
        )

        self.tY_pid = ProfiledPIDController(
            self.translation_kP,
            self.translation_kI,
            self.translation_kD,
            self.translation_profile,
        )

        self.tX_pid.setTolerance(0.02)
        self.tY_pid.setTolerance(0.02)

        self.r_pid = ProfiledPIDController(
            self.rotation_kP,
            self.rotation_kI,
            self.rotation_kD,
            self.rotation_profile,
        )

        self.r_pid.enableContinuousInput(-pi, pi)
        self.r_pid.setTolerance(pi / 36)  # 5 degrees

        self.addRequirements(self.drivetrain)

    def initialize(self) -> None:
        self.tX_pid.reset(self.drivetrain.get_state().pose.X())
        self.tY_pid.reset(self.drivetrain.get_state().pose.Y())
        self.r_pid.reset(self.drivetrain.get_state().pose.rotation().radians())

        if abs(self.drivetrain.get_operator_forward_direction().degrees() - 180) < 15:
            self.invert = True

    def execute(self) -> None:
        x = self.tX_pid.calculate(
            self.drivetrain.get_state().pose.X(), self.target_pose.X()
        )
        y = self.tY_pid.calculate(
            self.drivetrain.get_state().pose.Y(), self.target_pose.Y()
        )
        t = self.r_pid.calculate(
            self.drivetrain.get_state().pose.rotation().radians(),
            self.target_pose.rotation().radians(),
        )

        if self.invert:
            x *= -1
            y *= -1

        self.drivetrain.set_control(
            self._drive.with_velocity_x(x).with_velocity_y(y).with_rotational_rate(t)
        )

    def end(self, interrupted: bool):
        self.drivetrain.set_control(
            self._drive.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
        )

    def isFinished(self) -> bool:
        print(self.tX_pid.atGoal(), self.tY_pid.atGoal(), self.r_pid.atGoal())
        return self.tX_pid.atGoal() and self.tY_pid.atGoal() and self.r_pid.atGoal()
