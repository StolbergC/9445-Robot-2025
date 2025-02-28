from commands2 import Command, DeferredCommand, RunCommand, WaitCommand, InstantCommand
import commands2
from commands2.button import Trigger, CommandJoystick

from cscore import CameraServer
from ntcore import NetworkTableInstance
from wpilib import DriverStation, RobotBase
from wpilib import SmartDashboard, SendableChooser

from wpimath import applyDeadband
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.units import feetToMeters

from subsystems import elevator
from subsystems.drivetrain import Drivetrain
from subsystems.elevator import Elevator
from subsystems.wrist import Wrist
from subsystems.climber import Climber
from subsystems.claw import Claw
from subsystems.fingers import Fingers

from commands.score import score_coral
from commands.score_l1 import score_l1_on_true
from commands.score_l2 import score_l2_on_true
from commands.score_l3 import score_l3_on_true
from commands.intake import intake_algae_low, intake_algae_high, intake_coral

from auto import blue_left_four_coral, positions, blue_test

import wpilib.cameraserver

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

trigger_lt = 2


class RobotContainer:
    def __init__(self) -> None:
        self.nettable = NetworkTableInstance.getDefault().getTable("0000DriverInfo")
        self.alliance = DriverStation.Alliance.kBlue
        self.drivetrain = Drivetrain(self.get_alliance)
        self.wrist = Wrist()
        # self.climber = Climber()
        # self.claw = Claw(
        #     self.wrist.get_angle, Rotation2d.fromDegrees(60)
        # )  # TODO: Test the 60_deg. Should be as close to 90 as is safe.
        self.claw = Claw(lambda: Rotation2d(0), Rotation2d.fromDegrees(60))
        self.elevator = Elevator(lambda: Rotation2d(0))
        # self.wrist.get_claw_distance = self.claw.get_dist
        self.drivetrain.reset_pose(Pose2d(0, 0, Rotation2d(0)))
        # self.fingers = Fingers()

        self.auto_chooser = SendableChooser()
        self.auto_chooser.setDefaultOption("CHANGE ME", commands2.cmd.none())
        # self.auto_chooser.addOption(
        #     "Blue -- Four Coral Left", blue_left_four_coral.get_auto(self.drivetrain, self.elevator, self.wrist, self.claw,)
        # )
        self.auto_chooser.addOption("Blue -- Test", blue_test.get_auto(self.drivetrain))

        self.level = 1
        self.field_oriented = True

        # wpilib.cameraserver.CameraServer.launch()

        def pick_alliance(new_auto: Command):
            if "RED" in new_auto.getName().upper():
                self.alliance = DriverStation.Alliance.kRed
            elif "BLUE" in new_auto.getName().upper():
                self.alliance = DriverStation.Alliance.kBlue
            else:
                self.alliance = DriverStation.getAlliance()

        self.auto_chooser.onChange(pick_alliance)

        # have to read from elastic
        SmartDashboard.putData(self.auto_chooser)

        self.driver_controller = CommandJoystick(0)
        self.operator_controller = CommandJoystick(1)

        self.driver_controller.setYChannel(0)
        self.driver_controller.setXChannel(1)
        self.driver_controller.setTwistChannel(4)
        self.driver_controller.setThrottleChannel(3)

        self.operator_controller.setYChannel(0)
        self.operator_controller.setXChannel(1)
        self.operator_controller.setTwistChannel(4)
        self.operator_controller.setThrottleChannel(3)

        # this sets the motors to idle on disable
        Trigger(DriverStation.isEnabled).onTrue(
            self.drivetrain.set_drive_idle(False)
        ).onTrue(self.drivetrain.set_turn_idle(False))
        Trigger(DriverStation.isEnabled).onFalse(
            self.drivetrain.set_drive_idle(True)
        ).onFalse(self.drivetrain.set_turn_idle(True))

    # def get_reef_score_command(self) -> DeferredCommand:
    #     return DeferredCommand(
    #         lambda: (
    #             score_l1_on_true(self.elevator, self.wrist)
    #             if self.level == 1
    #             else (
    #                 score_l2_on_true(self.elevator, self.wrist)
    #                 if self.level == 2
    #                 else score_l3_on_true(self.elevator, self.wrist)
    #             )
    #         ),
    #         self.elevator,
    #         self.wrist,
    #         self.fingers,
    #         self.claw,
    #     )

    # def get_algae_intake_command(self) -> DeferredCommand:
    #     return DeferredCommand(
    #         lambda: (
    #             intake_algae_low(self.elevator, self.wrist, self.claw, self.fingers)
    #             if self.level == 1 or self.level == 2
    #             else intake_algae_high(
    #                 self.elevator, self.wrist, self.claw, self.fingers
    #             )
    #         ),
    #         self.elevator,
    #         self.wrist,
    #         self.fingers,
    #         self.claw,
    #     )

    def set_teleop_bindings(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.drive_joystick(
                lambda: -self.driver_controller.getX(),
                lambda: -self.driver_controller.getY(),
                lambda: -self.driver_controller.getTwist(),
                # this assumes that -1 is resting and 1 is full
                lambda: self.field_oriented,
            )
        )

        self.elevator.setDefaultCommand(
            RunCommand(
                lambda: self.elevator.motor.set(self.operator_controller.getX()),
                self.elevator,
            ),
        )
        self.operator_controller.button(button_lb).whileTrue(self.elevator.command_l1())

        self.operator_controller.button(button_right).onTrue(self.elevator.reset())

        self.driver_controller.button(button_b).onTrue(self.drivetrain.reset_gyro())

        def toggle_field_oriented():
            self.field_oriented = not self.field_oriented

        self.driver_controller.button(button_y).toggleOnTrue(
            InstantCommand(toggle_field_oriented)
        )

        # self.elevator.setDefaultCommand(
        #     RunCommand(
        #         lambda: self.elevator.manual_control(
        #             applyDeadband(self.operator_controller.getRawAxis(5), 0.05)
        #         ),
        #         self.elevator,
        #     )
        # )

        # self.elevator.setDefaultCommand(
        #     DeferredCommand(
        #         lambda: (
        #             self.elevator.command_l1()
        #             if self.level == 1
        #             else (
        #                 self.elevator.command_l2()
        #                 if self.level == 2
        #                 else self.elevator.command_l3()
        #             )
        #         ),
        #         self.elevator,
        #     )
        # )

        # Trigger(lambda: self.operator_controller.getThrottle() > 0.5).onTrue(
        #     self.get_reef_score_command()
        # ).onFalse(score(self.claw, self.fingers))

        self.driver_controller.button(button_a).whileTrue(
            self.drivetrain.drive_near_coral_station().alongWith()
        )

        self.driver_controller.button(button_lb).whileTrue(
            self.drivetrain.drive_closest_reef().alongWith(
                # self.get_reef_score_command()
            )
        )

        self.driver_controller.button(button_rb).whileTrue(
            self.drivetrain.drive_closest_algae().alongWith(
                # self.get_algae_intake_command()
            )
        )

        def increase_elevator_setpoint() -> None:
            self.level = (self.level % 3) + 1

        def decrease_elevator_setpoint() -> None:
            self.level = ((self.level + 1) % 3) + 1

        self.operator_controller.povUp().onTrue(
            InstantCommand(increase_elevator_setpoint)
        )

        self.operator_controller.povDown().onTrue(
            InstantCommand(decrease_elevator_setpoint)
        )

        # self.operator_controller.button(button_rb).onTrue(self.elevator.reset())

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

        Trigger(self.operator_controller.button(button_rb)).onTrue(
            self.claw.home_outside()
        )
        Trigger(self.operator_controller.button(button_a)).onTrue(self.claw.cage())
        Trigger(self.operator_controller.button(button_b)).onTrue(self.claw.coral())
        Trigger(self.operator_controller.button(button_x)).onTrue(self.claw.algae())

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

        # self.driver_controller.button(button_a).whileTrue(
        #     self.drivetrain.drive_position(Pose2d(0, 0, Rotation2d(0)))
        # )

        # self.driver_controller.button(button_b).onTrue(
        #     self.drivetrain.reset_pose(Pose2d())
        # )
        # self.operator_controller.button(button_rb).whileTrue(self.climber.climb())

        # self.driver_controller.button(button_y).whileTrue(
        #     self.drivetrain.drive_position(
        #         Pose2d.fromFeet(0, 0, Rotation2d.fromDegrees(0))
        #     )
        #     .andThen(WaitCommand(0.5))
        #     .andThen(
        #         self.drivetrain.drive_position(
        #             Pose2d.fromFeet(4, 0, Rotation2d.fromDegrees(90))
        #         )
        #     )
        #     .andThen(WaitCommand(0.5))
        #     .andThen(
        #         self.drivetrain.drive_position(
        #             Pose2d.fromFeet(4, 4, Rotation2d.fromDegrees(0))
        #         )
        #     )
        # )

    def periodic(self) -> None:
        self.nettable.putNumber("Elevator Level", self.level)

    def get_alliance(self) -> DriverStation.Alliance:
        return self.alliance

    def get_auto_command(self) -> Command:
        return self.auto_chooser.getSelected()
        # return blue_left_four_coral.get_auto(self.drivetrain)
