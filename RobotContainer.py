from commands2 import Command, RunCommand, WaitCommand, InstantCommand
import commands2
from commands2.button import Trigger, CommandJoystick

from wpilib import DriverStation, RobotBase
from wpilib import SmartDashboard, SendableChooser

from wpimath import applyDeadband
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.units import feetToMeters

from subsystems.drivetrain import Drivetrain
from subsystems.wrist import Wrist
from subsystems.climber import Climber
from subsystems.claw import Claw
from subsystems.fingers import Fingers

from auto import blue_left_four_coral, positions, blue_test

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
        # self.claw = Claw(
        # self.wrist.get_angle, Rotation2d.fromDegrees(60)
        # )  # TODO: Test the 60_deg. Should be as close to 90 as is safe.
        # self.claw = Claw(lambda: Rotation2d(0), Rotation2d.fromDegrees(60))
        self.drivetrain.reset_pose(Pose2d(0, 0, Rotation2d(0)))
        # self.fingers = Fingers()

        self.auto_chooser = SendableChooser()
        self.auto_chooser.setDefaultOption("CHANGE ME", commands2.cmd.none())
        self.auto_chooser.addOption(
            "Blue -- Four Coral Left", blue_left_four_coral.get_auto(self.drivetrain)
        )
        self.auto_chooser.addOption("Blue -- Test", blue_test.get_auto(self.drivetrain))

        # have to read from elastic
        SmartDashboard.putData(self.auto_chooser)

        self.driver_controller = CommandJoystick(0)
        self.operator_controller = CommandJoystick(1)

        self.driver_controller.setYChannel(0)
        self.driver_controller.setXChannel(1)
        self.driver_controller.setTwistChannel(4)
        self.driver_controller.setThrottleChannel(3)

        self.driver_controller.setYChannel(0)
        self.driver_controller.setXChannel(1)
        self.driver_controller.setTwistChannel(4)
        self.driver_controller.setThrottleChannel(3)

        # this sets the motors to idle on disable
        # Trigger(DriverStation.isEnabled).onTrue(
        #     self.drivetrain.set_drive_idle(False)
        # ).onTrue(self.drivetrain.set_turn_idle(False))
        # Trigger(DriverStation.isEnabled).onFalse(
        #     self.drivetrain.set_drive_idle(True)
        # ).onFalse(self.drivetrain.set_turn_idle(True))

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
        # self.wrist.setDefaultCommand(
        #     RunCommand(
        #         lambda: self.wrist.motor.set(self.operator_controller.getTwist()),
        #         self.wrist,
        #     )
        # )

        # self.claw.setDefaultCommand(
        #     RunCommand(
        #         lambda: self.claw.set_motor(
        #             applyDeadband(self.operator_controller.getX(), 0.05)
        #         ),
        #         self.claw,
        #     )
        # )

        # self.fingers.setDefaultCommand(self.fingers.stop())

        # self.operator_controller.button(button_left).whileTrue(self.fingers.intake())
        # self.operator_controller.button(button_right).whileTrue(self.fingers.score())

        self.driver_controller.button(button_x).onTrue(
            self.drivetrain.drive_position(positions.blue_coral_intake_right_left)
        )

        # Trigger(self.operator_controller.button(button_rb)).onTrue(self.claw.reset())
        # Trigger(self.operator_controller.button(button_a)).onTrue(self.claw.cage())
        # Trigger(self.operator_controller.button(button_b)).onTrue(self.claw.coral())
        # Trigger(self.operator_controller.button(button_x)).onTrue(self.claw.algae())

        self.driver_controller.button(button_lpush).or_(
            self.driver_controller.button(button_rpush)
        ).whileTrue(self.drivetrain.defense_mode())

        # self.operator_controller.button(button_a).onTrue(self.wrist.angle_score())

        # self.operator_controller.button(button_b).onTrue(self.wrist.angle_zero())

        # self.operator_controller.button(button_y).onTrue(self.wrist.angle_intake())

        lb_trigger = self.operator_controller.button(button_lb)
        rb_trigger = self.operator_controller.button(button_rb)

        # drive to the algae on the closest reef when lb and rb are pressed
        lb_trigger.and_(rb_trigger).whileTrue(WaitCommand(0))

        # drive to the left peg on the closest part of the reef when only lb is pressed
        lb_trigger.and_(rb_trigger.not_()).whileTrue(WaitCommand(0))

        # drive to the right peg on the closest part of the reef when only lb is pressed
        rb_trigger.and_(lb_trigger.not_()).whileTrue(WaitCommand(0))

        self.driver_controller.button(button_a).whileTrue(
            self.drivetrain.drive_position(Pose2d(0, 0, Rotation2d(0)))
        )

        self.driver_controller.button(button_b).onTrue(
            self.drivetrain.reset_pose(Pose2d())
        )
        # self.operator_controller.button(button_rb).whileTrue(self.climber.climb())

        self.driver_controller.button(button_y).whileTrue(
            self.drivetrain.drive_position(
                Pose2d.fromFeet(0, 0, Rotation2d.fromDegrees(0))
            )
            .andThen(WaitCommand(0.5))
            .andThen(
                self.drivetrain.drive_position(
                    Pose2d.fromFeet(4, 0, Rotation2d.fromDegrees(90))
                )
            )
            .andThen(WaitCommand(0.5))
            .andThen(
                self.drivetrain.drive_position(
                    Pose2d.fromFeet(4, 4, Rotation2d.fromDegrees(0))
                )
            )
        )

    def get_auto_command(self) -> Command:
        return self.auto_chooser.getSelected()
        # return blue_left_four_coral.get_auto(self.drivetrain)
