from commands2 import Command, RunCommand, WaitCommand, InstantCommand
from commands2.button import Trigger, CommandJoystick

from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.units import feetToMeters
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig

from subsystems.drivetrain import Drivetrain
from subsystems.wrist import Wrist
from subsystems.climber import Climber

from auto import blue_blue, positions, red_red

button_a = 1
button_b = 2
button_x = 3
button_y = 4
button_lb = 5
button_rb = 6
button_left = 7
button_right = 8
button_lpush = 9
button_rpush = 10


class RobotContainer:
    def __init__(self) -> None:
        self.drivetrain = Drivetrain()
        # self.wrist = Wrist()
        # self.climber = Climber()
        self.drivetrain.reset_pose(Pose2d(0, 0, Rotation2d(0)))

        self.driver_controller = CommandJoystick(0)
        self.operator_controller = self.driver_controller
        # self.operator_controller = Joystick(1)

        self.driver_controller.setYChannel(0)
        self.driver_controller.setXChannel(1)
        self.driver_controller.setTwistChannel(4)
        self.driver_controller.setThrottleChannel(3)

        # this sets the motors to idle on disable
        Trigger(DriverStation.isEnabled).onTrue(
            self.drivetrain.set_drive_idle(False)
        ).onTrue(self.drivetrain.set_turn_idle(False))
        Trigger(DriverStation.isEnabled).onFalse(
            self.drivetrain.set_drive_idle(True)
        ).onFalse(self.drivetrain.set_turn_idle(True))

    def set_teleop_bindings(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.drive_joystick(
                lambda: -self.driver_controller.getX(),
                lambda: -self.driver_controller.getY(),
                lambda: -self.driver_controller.getTwist(),
                # this assumes that -1 is resting and 1 is full
                lambda: self.driver_controller.getThrottle() > 0,
            )
        )

        self.driver_controller.button(button_lpush).or_(
            self.driver_controller.button(button_rpush)
        ).whileTrue(self.drivetrain.defense_mode())

        # self.operator_controller.button(button_a).onTrue(self.wrist.angle_score())

        # self.operator_controller.button(button_b).onTrue(self.wrist.angle_zero())

        # self.operator_controller.button(button_y).onTrue(self.wrist.angle_intake())

        # lb_trigger = self.operator_controller.button(button_lb)
        # rb_trigger = self.operator_controller.button(button_rb)

        # drive to the algae on the closest reef when lb and rb are pressed
        # lb_trigger.and_(rb_trigger).whileTrue(WaitCommand(0))

        # drive to the left peg on the closest part of the reef when only lb is pressed
        # lb_trigger.and_(rb_trigger.not_()).whileTrue(WaitCommand(0))

        # drive to the right peg on the closest part of the reef when only lb is pressed
        # rb_trigger.and_(lb_trigger.not_()).whileTrue(WaitCommand(0))

        # self.driver_controller.button(button_a).whileTrue(
        #     self.drivetrain.drive_forward(1)
        # )
        self.driver_controller.button(button_a).whileTrue(
            self.drivetrain.drive_position(Pose2d(0, 0, Rotation2d(0)))
        )

        # self.driver_controller.button(button_b).onTrue(self.drivetrain.reset_pose(Pose2d()))
        # self.operator_controller.button(button_rb).whileTrue(self.climber.climb())

        self.driver_controller.button(button_b).whileTrue(
            self.drivetrain.reset_pose(Pose2d(0, 0, Rotation2d(0)))
        )

        self.driver_controller.button(button_y).whileTrue(
            self.drivetrain.drive_position(
                Pose2d.fromFeet(0, 0, Rotation2d.fromDegrees(0))
            )
            .andThen(WaitCommand(0.5))
            .andThen(
                self.drivetrain.drive_position(
                    Pose2d.fromFeet(4, 0, Rotation2d.fromDegrees(0))
                )
            )
            .andThen(WaitCommand(0.5))
            .andThen(
                self.drivetrain.drive_position(
                    Pose2d.fromFeet(4, 4, Rotation2d.fromDegrees(0))
                )
            )
        )

        self.driver_controller.button(button_rb).whileTrue(
            red_red.get_auto(self.drivetrain)
        )

        self.driver_controller.button(button_lb).whileTrue(
            blue_blue.get_auto(self.drivetrain)
        )
        # cfg = TrajectoryConfig(
        #     self.drivetrain.max_velocity_mps,
        #     self.drivetrain.max_velocity_mps * 10,
        # )
        # self.driver_controller.button(button_y).whileTrue(
        #     self.drivetrain.drive_trajectory(
        #         TrajectoryGenerator.generateTrajectory(
        #             Pose2d.fromFeet(0, 0, Rotation2d.fromDegrees(0)),
        #             [
        #                 Translation2d.fromFeet(5, 0),
        #             ],  # , Translation2d.fromFeet(7, 3)],
        #             Pose2d.fromFeet(5, 5, Rotation2d.fromDegrees(90)),
        #             cfg,
        #         )
        #     ).andThen(
        #         self.drivetrain.drive_trajectory(
        #             TrajectoryGenerator.generateTrajectory(
        #                 Pose2d.fromFeet(5, 5, Rotation2d.fromDegrees(90)),
        #                 [Translation2d.fromFeet(0, 5)],
        #                 Pose2d.fromFeet(0, 0, Rotation2d.fromDegrees(0)),
        #                 cfg,
        #             )
        #         )
        #     )
        # )

    def unset_teleop_bindings(self) -> None:
        self.drivetrain.setDefaultCommand(WaitCommand(0))
        # self.wrist.setDefaultCommand(WaitCommand(0))

    def get_auto_command(self) -> Command:
        return blue_blue.get_auto(self.drivetrain)
