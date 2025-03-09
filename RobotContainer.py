from commands2 import (
    Command,
    DeferredCommand,
    RepeatCommand,
    RunCommand,
    WaitCommand,
    InstantCommand,
    WrapperCommand,
)
import commands2
from commands2.button import Trigger, CommandJoystick

from cscore import CameraServer
from ntcore import NetworkTableInstance
from wpilib import DriverStation, RobotBase
from wpilib import SmartDashboard, SendableChooser, PowerDistribution

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
from commands.intake import (
    intake_algae_low,
    intake_algae_high,
    intake_coral,
    intake_algae_ground,
)

from auto import (
    blue_center_two_algae,
    blue_left_two_coral,
    blue_right_two_coral,
    positions,
    blue_test,
    red_center_two_algae,
    red_left_two_coral,
    red_right_two_coral,
)

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
        self.pdh = PowerDistribution()
        self.pdh.setSwitchableChannel(True)
        self.nettable = NetworkTableInstance.getDefault().getTable("0000DriverInfo")
        a = DriverStation.getAlliance()
        if a is None:
            self.alliance = DriverStation.Alliance.kBlue
        else:
            self.alliance = a
        self.drivetrain = Drivetrain(self.get_alliance)
        self.wrist = Wrist()
        # self.climber = Climber()
        # self.claw = Claw(
        #     self.wrist.get_angle, Rotation2d.fromDegrees(60)
        # )  # TODO: Test the 60_deg. Should be as close to 90 as is safe.
        self.claw = Claw(lambda: Rotation2d(0), Rotation2d.fromDegrees(60))
        self.elevator = Elevator(lambda: Rotation2d(0))
        # self.wrist.get_claw_distance = self.claw.get_dist
        self.wrist.get_claw_distance = lambda: 0
        self.drivetrain.reset_pose(Pose2d(0, 0, Rotation2d(0)))
        self.fingers = Fingers()

        self.auto_chooser = SendableChooser()
        self.auto_chooser.setDefaultOption("CHANGE ME", commands2.cmd.none())
        # self.auto_chooser.addOption(
        #     "Blue -- Four Coral Left", blue_left_two_coral.get_auto(self.drivetrain, self.elevator, self.wrist, self.claw,)
        # )
        self.auto_chooser.addOption(
            "Blue -- Coral Left",
            blue_left_two_coral.get_auto(
                self.drivetrain, self.elevator, self.wrist, self.claw, self.fingers
            ),
        )
        self.auto_chooser.addOption(
            "Blue -- Coral Right",
            blue_right_two_coral.get_auto(
                self.drivetrain, self.elevator, self.wrist, self.claw, self.fingers
            ),
        )
        self.auto_chooser.addOption(
            "Blue -- Algae",
            blue_center_two_algae.get_auto(
                self.drivetrain, self.elevator, self.wrist, self.claw, self.fingers
            ),
        )
        self.auto_chooser.addOption(
            "Red -- Coral Left",
            red_left_two_coral.get_auto(
                self.drivetrain, self.elevator, self.wrist, self.claw, self.fingers
            ),
        )
        self.auto_chooser.addOption(
            "Red -- Coral Right",
            red_right_two_coral.get_auto(
                self.drivetrain, self.elevator, self.wrist, self.claw, self.fingers
            ),
        )
        self.auto_chooser.addOption(
            "Red -- Algae",
            red_center_two_algae.get_auto(
                self.drivetrain, self.elevator, self.wrist, self.claw, self.fingers
            ),
        )

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

        self.grabbing_coral = True

        # this sets the motors to idle on disable
        Trigger(DriverStation.isEnabled).onTrue(
            self.drivetrain.set_drive_idle(False)
        ).onTrue(self.drivetrain.set_turn_idle(False))
        Trigger(DriverStation.isEnabled).onFalse(
            self.drivetrain.set_drive_idle(True)
        ).onFalse(self.drivetrain.set_turn_idle(True))

    def get_reef_score_command(self) -> DeferredCommand:
        return DeferredCommand(
            lambda: (
                score_l1_on_true(self.elevator, self.wrist)
                if self.level == 1
                else (
                    score_l2_on_true(self.elevator, self.wrist)
                    if self.level == 2
                    else score_l3_on_true(self.elevator, self.wrist)
                )
            ),
            self.elevator,
            self.wrist,
            self.fingers,
            self.claw,
        )

    def get_algae_intake_command(self) -> DeferredCommand:
        return DeferredCommand(
            lambda: (
                intake_algae_ground(self.elevator, self.wrist, self.claw)
                if self.level == 1
                else (
                    intake_algae_low(self.elevator, self.wrist, self.claw)
                    if self.level == 2
                    else intake_algae_high(self.elevator, self.wrist, self.claw)
                )
            ),
            self.elevator,
            self.wrist,
            self.fingers,
            self.claw,
        )

    def get_score_command(self) -> DeferredCommand:
        return DeferredCommand(
            lambda: (
                self.get_reef_score_command()
                if self.grabbing_coral
                else self.wrist.angle_zero().andThen(self.elevator.command_processor())
            ),
            self.elevator,
            self.wrist,
            self.claw,
            self.fingers,
        )

    def get_intake_command(self) -> DeferredCommand:
        return DeferredCommand(
            lambda: (
                intake_coral(self.elevator, self.wrist, self.claw, self.fingers)
                if self.grabbing_coral
                else self.get_algae_intake_command()
            )
        )

    def get_intake_on_false(self) -> DeferredCommand:
        return DeferredCommand(
            lambda: self.claw.coral() if self.grabbing_coral else self.claw.algae()
        )

    def set_teleop_bindings(self) -> None:
        """testing"""
        # self.wrist.setDefaultCommand(
        #     RepeatCommand(
        #         self.wrist.angle_score()
        #         .andThen(WaitCommand(0.25))
        #         .andThen(self.wrist.angle_intake())
        #     )
        # )

        # self.elevator.setDefaultCommand(
        #     # self.elevator.command_processor()
        #     RepeatCommand(
        #         self.elevator.command_l1()
        #         .andThen(WaitCommand(0.25))
        #         .andThen(self.elevator.command_position(0))
        #         .andThen(WaitCommand(0.25))
        #         .andThen(self.elevator.command_l2())
        #         .andThen(WaitCommand(0.25))
        #         .andThen(self.elevator.command_position(0))
        #         .andThen(WaitCommand(0.25))
        #         .andThen(self.elevator.command_l3())
        #         .andThen(WaitCommand(0.25))
        #         .andThen(self.elevator.command_position(0))
        #         .andThen(WaitCommand(0.25))
        #     )
        # )

        self.elevator.setDefaultCommand(
            RunCommand(
                lambda: self.elevator.manual_control(
                    applyDeadband(self.operator_controller.getX(), 0.1)
                ),
                self.elevator,
            ),
        )
        self.operator_controller.button(button_x).onTrue(self.elevator.reset())

        """actual bindings"""
        """defaults"""
        self.drivetrain.setDefaultCommand(
            self.drivetrain.drive_joystick(
                lambda: applyDeadband(-self.driver_controller.getX(), 0.05),
                lambda: applyDeadband(-self.driver_controller.getY(), 0.05),
                lambda: applyDeadband(-self.driver_controller.getTwist(), 0.05),
                # this assumes that -1 is resting and 1 is full
                lambda: self.field_oriented,
            )
        )

        self.wrist.setDefaultCommand(self.wrist.follow_angle())

        """driver"""
        self.driver_controller.button(button_b).onTrue(self.drivetrain.reset_gyro())

        def toggle_field_oriented():
            self.field_oriented = not self.field_oriented

        self.driver_controller.button(button_y).onTrue(
            InstantCommand(toggle_field_oriented)
        )

        self.driver_controller.button(button_a).whileTrue(
            self.drivetrain.drive_near_coral_station().alongWith(
                intake_coral(self.elevator, self.wrist, self.claw, self.fingers)
            )
        )

        self.driver_controller.button(button_lb).whileTrue(
            self.drivetrain.drive_closest_reef().alongWith(
                self.get_reef_score_command()
            )
        )

        self.driver_controller.button(button_rb).whileTrue(
            self.drivetrain.drive_closest_algae().alongWith(
                self.get_algae_intake_command()
            )
        )

        """operator controls"""
        Trigger(lambda: self.operator_controller.getThrottle() > 0.5).whileTrue(
            self.get_score_command()
        ).onFalse(self.fingers.score().withTimeout(2).andThen(self.fingers.stop()))

        Trigger(
            lambda: self.operator_controller.getRawAxis(trigger_lt) > 0.5
        ).whileTrue(self.get_intake_command()).onFalse(
            self.get_intake_on_false().andThen(
                WaitCommand(1).andThen(self.fingers.stop().alongWith(self.claw.stop()))
            )
        )

        def increase_elevator_setpoint() -> None:
            self.level += 1
            if self.level > 3:
                self.level = 3

        def decrease_elevator_setpoint() -> None:
            self.level -= 1
            if self.level < 1:
                self.level = 1

        self.operator_controller.povUp().onTrue(
            InstantCommand(increase_elevator_setpoint)
        )

        self.operator_controller.povDown().onTrue(
            InstantCommand(decrease_elevator_setpoint)
        )

        def set_piece(coral: bool) -> None:
            self.grabbing_coral = coral

        self.operator_controller.button(button_a).onTrue(
            InstantCommand(lambda: set_piece(False))
        )

        self.operator_controller.button(button_y).onTrue(
            InstantCommand(lambda: set_piece(True))
        )

        # self.operator_controller.povRight().onTrue(self.wrist.command_zero())

        # self.operator_controller.button(button_b).onTrue(self.wrist.command_intake())

        self.operator_controller.button(button_b).onTrue(
            self.wrist.angle_zero()
            .andThen(self.elevator.command_bottom())
            .andThen(self.wrist.angle_intake())
        )

    def periodic(self) -> None:
        self.nettable.putNumber("Elevator Level", self.level)
        self.nettable.putBoolean("Coral", self.grabbing_coral)

    def get_alliance(self) -> DriverStation.Alliance:
        return self.alliance

    def get_auto_command(self) -> Command:
        return self.auto_chooser.getSelected()
        # return blue_left_two_coral.get_auto(self.drivetrain)
