from commands2 import Command, InstantCommand
import commands2
from phoenix6 import swerve
from wpimath import applyDeadband
from telemetry import Telemetry
from generated.tuner_constants import TunerConstants

from commands2.button import CommandXboxController

from ntcore import NetworkTableInstance
from ntcore.util import ntproperty

from wpilib import PowerDistribution, DriverStation, SmartDashboard

from pathplannerlib.auto import AutoBuilder

from subsystems.elevator import Elevator
from subsystems.leds import Leds
from subsystems.wrist import Wrist
from subsystems.climber import Climber
from subsystems.claw import Claw
from subsystems.fingers import Fingers

from commands.score_l1 import score_l1_on_true
from commands.score_l2 import score_l2_on_true
from commands.score_l3 import score_l3_on_true
from commands.elevator_manual import ElevatorManual
from commands.intake import intake_coral
from commands.elevator_bottom import ElevatorBottom
from commands.wrist_angle_zero import WristZero
from commands.wrist_intake import WristIntake
from commands.wrist_l1 import WristL1
from commands.wrist_l2 import WristL2
from commands.wrist_l3 import WristL3
from commands.claw_coral import ClawCoral
from commands.claw_neutral import ClawNeutral
from commands.fingers_score import FingersScore
from commands.fingers_stop import FingersStop


class RobotContainer:
    _max_speed_percent = ntproperty("MaxVelocityPercent", 1)
    _max_angular_rate_percent = ntproperty("MaxOmegaPercent", 1)

    _max_speed = TunerConstants.speed_at_12_volts
    _max_angular_rate = 0.75  # radians per second

    def __init__(self) -> None:
        self.driver_controller = CommandXboxController(0)
        self.operator_controller = CommandXboxController(1)
        self.pdh = PowerDistribution()
        self.pdh.setSwitchableChannel(True)
        self.nettable = NetworkTableInstance.getDefault().getTable("0000DriverInfo")

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = swerve.requests.FieldCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )  # Use open-loop control for drive motors

        self._robot_drive = swerve.requests.RobotCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )  # Use open-loop control for drive motors

        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

        self.drivetrain = TunerConstants.create_drivetrain()
        self.wrist = Wrist()
        self.climber = Climber()
        self.claw = Claw()
        self.elevator = Elevator()
        self.fingers = Fingers()

        self.leds = Leds()

        self.auto_chooser = AutoBuilder.buildAutoChooser()

        SmartDashboard.putData(self.auto_chooser)

    def get_velocity_x(self) -> float:
        x = applyDeadband(self.driver_controller.getLeftX(), 0.05)
        return x * abs(x) * self._max_speed * self._max_speed_percent

    def get_velocity_y(self) -> float:
        y = applyDeadband(self.driver_controller.getLeftY(), 0.05)
        return y * abs(y) * self._max_speed * self._max_speed_percent

    def get_angular_rate(self) -> float:
        t = applyDeadband(self.driver_controller.getRightY(), 0.05)
        return t * abs(t) * self._max_angular_rate * self._max_angular_rate_percent

    def set_teleop_bindings(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: self._drive.with_velocity_x(self.get_velocity_x())
                .with_velocity_y(self.get_velocity_y())
                .with_rotational_rate(self.get_angular_rate())
            )
        )

        # robot oriented on LB Hold
        self.driver_controller.leftBumper().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._robot_drive.with_velocity_x(self.get_velocity_x())
                .with_velocity_y(self.get_velocity_y())
                .with_rotational_rate(self.get_angular_rate())
            )
        )

        # slow mode and defense mode
        def half_speed():
            self._max_speed_percent /= 2
            self._max_angular_rate_percent /= 2

        def double_speed():
            self._max_speed_percent *= 2
            self._max_angular_rate_percent *= 2

        # slow mode
        self.driver_controller.leftTrigger().onTrue(InstantCommand(half_speed)).onFalse(
            InstantCommand(double_speed)
        )

        # defense mode
        self.driver_controller.rightTrigger().onTrue(
            InstantCommand(double_speed)
        ).onFalse(InstantCommand(half_speed))

    def set_test_bindings(self) -> None:
        # will be sysid testing for drivetrain (+others?) sometime
        self.test_remote = CommandXboxController(2)

    def get_auto_command(self) -> Command:
        return commands2.cmd.none()

    def get_auto_name(self) -> str:
        return ""
