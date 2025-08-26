from commands2 import (
    Command,
    ConditionalCommand,
    DeferredCommand,
    RepeatCommand,
    RunCommand,
    Subsystem,
    WaitCommand,
    InstantCommand,
    WrapperCommand,
)
import commands2
from commands2.button import Trigger, CommandJoystick

from cscore import CameraServer
from ntcore import NetworkTableInstance
from pathplannerlib.auto import NamedCommands, EventTrigger
import pathplannerlib
import pathplannerlib.pathfinders
from wpilib import DriverStation, RobotBase
from wpilib import SmartDashboard, SendableChooser, PowerDistribution

from wpimath import applyDeadband
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.units import feetToMeters

from pathplannerlib.auto import AutoBuilder, PathConstraints

from commands.elevator_manual import ElevatorManual
from subsystems.drivetrain import Drivetrain
from subsystems.elevator import Elevator
from subsystems.leds import Leds
from subsystems.wrist import Wrist
from subsystems.climber import Climber
from subsystems.claw import Claw
from subsystems.fingers import Fingers

from commands.score import score_coral
from commands.score_l1 import score_l1_on_true
from commands.score_l2 import score_l2_on_true
from commands.score_l3 import score_l3_on_true
from commands.intake import intake_coral
from commands.wrist_angle_slow import WristAngleSlow
from commands.elevator_bottom import ElevatorBottom

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


class FakeSubsystem(Subsystem): ...


class RobotContainer:
    def __init__(self) -> None:
        self._fake_subsystem = FakeSubsystem()
        self.pdh = PowerDistribution()
        self.pdh.setSwitchableChannel(True)
        self.nettable = NetworkTableInstance.getDefault().getTable("0000DriverInfo")
        a = DriverStation.getAlliance()
        if a is None:
            self.alliance = DriverStation.Alliance.kBlue
        else:
            self.alliance = a
        self.drivetrain = Drivetrain()
        self.wrist = Wrist()
        self.climber = Climber()
        self.claw = Claw(
            lambda: Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(68),
            Rotation2d.fromDegrees(55),
        )
        # self.elevator = Elevator(lambda: Rotation2d(0))  # self.wrist.get_angle)
        self.elevator = Elevator()
        self.wrist.get_claw_distance = lambda: 0  # self.claw.get_dist
        self.wrist.safe_claw_distance = 10
        # self.drivetrain.reset_pose(Pose2d(0, 0, Rotation2d(0)))
        self.fingers = Fingers()

        self.leds = Leds()

        """
        NamedCommands.registerCommand("FinishScore", score_coral(self.fingers, 2))
        # I forget the safe angle
        EventTrigger("StartPinchCoral").onTrue(
            WristAngleSlow(self.wrist, Rotation2d.fromDegrees(60)).andThen(
                self.claw.coral()
            )
        )

        EventTrigger("ReadyL2").onTrue(score_l3_on_true(self.elevator, self.wrist))
        EventTrigger("Stow").onTrue(self.get_stow())
        EventTrigger("WristAnglePreload").onTrue(
            self.wrist.run_angle(Rotation2d.fromDegrees(90))
        )

        self.auto_chooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.auto_chooser)

        # self.auto_chooser.setDefaultOption("CHANGE ME", commands2.cmd.none())
        # self.auto_chooser.addOption(
        #     "Blue -- Four Coral Left", blue_left_two_coral.get_auto(self.drivetrain, self.elevator, self.wrist, self.claw,)
        # )
        # self.auto_chooser.addOption(
        #     "Blue -- Coral Left",
        #     blue_left_two_coral.get_auto(
        #         self.drivetrain, self.elevator, self.wrist, self.claw, self.fingers
        #     ),
        # )
        # self.auto_chooser.addOption(
        #     "Blue -- Coral Right",
        #     blue_right_two_coral.get_auto(
        #         self.drivetrain, self.elevator, self.wrist, self.claw, self.fingers
        #     ),
        # )
        # self.auto_chooser.addOption(
        #     "Blue -- Algae",
        #     blue_center_two_algae.get_auto(
        #         self.drivetrain, self.elevator, self.wrist, self.claw, self.fingers
        #     ),
        # )

        # self.auto_chooser.addOption(
        #     "Blue -- Drive", blue_drive.get_auto(self.drivetrain)
        # )

        # self.auto_chooser.addOption(
        #     "Red -- Coral Left",
        #     red_left_two_coral.get_auto(
        #         self.drivetrain, self.elevator, self.wrist, self.claw, self.fingers
        #     ),
        # )
        # self.auto_chooser.addOption(
        #     "Red -- Coral Right",
        #     red_right_two_coral.get_auto(
        #         self.drivetrain, self.elevator, self.wrist, self.claw, self.fingers
        #     ),
        # )
        # self.auto_chooser.addOption(
        #     "Red -- Algae",
        #     red_center_two_algae.get_auto(
        #         self.drivetrain, self.elevator, self.wrist, self.claw, self.fingers
        #     ),
        # )

        # self.auto_chooser.addOption("Red Drive", red_drive.get_auto(self.drivetrain))
        """

        self.level = 1
        self.field_oriented = True

        # def pick_alliance(new_auto: Command):
        # if "RED" in new_auto.getName().upper():
        # self.alliance = DriverStation.Alliance.kRed
        # elif "BLUE" in new_auto.getName().upper():
        # self.alliance = DriverStation.Alliance.kBlue
        # else:
        # self.alliance = DriverStation.getAlliance()

        # self.auto_chooser.onChange(pick_alliance)

        # SmartDashboard.putData(self.auto_chooser)

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
        # Trigger(DriverStation.isEnabled).onTrue(
        #     self.drivetrain.set_drive_idle_command(False).andThen(
        #         InstantCommand(lambda: self.pdh.setSwitchableChannel(False))
        #     )
        # ).onFalse(
        #     (
        #         WaitCommand(5)
        #         .andThen(self.drivetrain.set_drive_idle_command(True))
        #         .andThen(self.drivetrain.set_turn_idle_command(True))
        #         .andThen(InstantCommand(lambda: self.pdh.setSwitchableChannel(True)))
        #     ).ignoringDisable(True)
        # )

        # self.claw.stop().schedule()
        # wpilib.cameraserver.CameraServer().launch()

        # self.fingers.setDefaultCommand(self.fingers.stop())
        # self.climber.setDefaultCommand(self.climber.stop())
        # self.wrist.setDefaultCommand(self.wrist.default_follow_ff().withName("Feed"))
        # self.wrist.setDefaultCommand(self.wrist.follow_angle())

        self.invert = 1

    def get_reef_score_command(self) -> WrapperCommand:
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
        ).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)

    def get_intake_on_false(self) -> Command:
        return self.claw.coral().withInterruptBehavior(
            Command.InterruptionBehavior.kCancelSelf
        )

    def get_stow(self) -> Command:
        return (
            self.wrist.angle_zero()
            .andThen(self.claw.cage())
            .andThen(ElevatorBottom(self.elevator))
            .andThen(self.wrist.angle_intake())
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        )

    def get_drive_x(self) -> float:
        return (
            self.invert
            * applyDeadband(-self.driver_controller.getX(), 0.1)
            * abs(self.driver_controller.getX())
        )

    def get_drive_y(self) -> float:
        return (
            self.invert
            * applyDeadband(-self.driver_controller.getY(), 0.1)
            * abs(self.driver_controller.getY())
        )

    def get_drive_t(self) -> float:
        return applyDeadband(self.driver_controller.getTwist(), 0.1) * abs(
            self.driver_controller.getTwist()
        )

    def set_teleop_bindings(self) -> None:
        """testing"""

        def make_pathfind() -> Command:
            pose = self.drivetrain.get_pose()
            # outside of field
            if pose.X() < 0 or pose.X() > 17.76 or pose.Y() < 0 or pose.Y() > 8.06:
                return commands2.cmd.none()
            out = AutoBuilder.pathfindToPose(
                Pose2d(0, 6, Rotation2d(0)),
                PathConstraints(
                    v := self.drivetrain.max_speed,
                    10 * v,
                    t := self.drivetrain.max_angular_speed,
                    10 * t,
                    # unlimited=True,
                ),
            )
            out.addRequirements(self.drivetrain)
            return out

        Trigger(lambda: self.driver_controller.getThrottle() > 0.5).whileTrue(
            DeferredCommand(
                make_pathfind,
                self.drivetrain,
            )
            .withName("path")
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        )

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

        # self.elevator.setDefaultCommand(self.elevator.stop())

        # enable manual control of the elevator
        Trigger(lambda: abs(self.operator_controller.getX()) > 0.1).whileTrue(
            ElevatorManual(self.elevator, self.operator_controller.getX)
        ).onFalse(InstantCommand(lambda: self.elevator.stop()))

        self.claw.setDefaultCommand(self.claw.stop())
        self.operator_controller.button(button_x).onTrue(
            InstantCommand(lambda: self.elevator.reset_position(0))
        )
        # self.operator_controller.button(button_b).onTrue(
        #     self.elevator.reset(self.elevator.top_height)
        # )

        """actual bindings"""
        """defaults"""

        # """driver"""
        # self.driver_controller.button(button_b).onTrue(
        #     self.drivetrain.reset_gyro_command(Rotation2d())
        # )

        def toggle_field_oriented():
            print("Toggling field oriented")
            self.field_oriented = not self.field_oriented

        self.driver_controller.button(button_y).onTrue(
            DeferredCommand(lambda: InstantCommand(toggle_field_oriented))
        )

        self.driver_controller.button(button_b).onTrue(
            self.drivetrain.reset_gyro_command(Rotation2d())
            # InstantCommand(lambda: self.drivetrain.reset_pose(Pose2d()))
        )

        # self.driver_controller.button(button_a).whileTrue(
        #     self.drivetrain.drive_near_coral_station().alongWith(
        #         intake_coral(self.elevator, self.wrist, self.claw, self.fingers)
        #     )
        # )

        # Trigger(lambda: self.driver_controller.getThrottle() > 0.5).onTrue(
        #     self.drivetrain.set_speed_command(
        #         feetToMeters(7), self.drivetrain.max_angular_velocity
        #     )
        # ).onFalse(
        #     DeferredCommand(
        #         lambda: self.drivetrain.set_speed_command(
        #             # self.drivetrain.old_speed, self.drivetrain.old_rot
        #             12,
        #             Rotation2d.fromDegrees(180),
        #         ),
        #         self.drivetrain,
        #     )
        # )

        # self.driver_controller.button(button_lb).whileTrue(
        #     self.drivetrain.drive_closest_reef().alongWith(
        #         self.get_reef_score_command()
        #     )
        # )

        # self.driver_controller.button(button_rb).whileTrue(
        #     # self.drivetrain.drive_closest_algae().alongWith(
        #     #     self.get_algae_intake_command()
        #     # )
        #     self.drivetrain.reset_pose(positions.blue_reef_center)
        # )

        # Trigger(lambda: self.driver_controller.getRawAxis(trigger_lt) > 0.5).onTrue(
        #     # self.drivetrain.auto_rotate_joystick_drive(
        #     #     lambda: applyDeadband(-self.driver_controller.getX(), 0.05),
        #     #     lambda: applyDeadband(-self.driver_controller.getY(), 0.05),
        #     #     lambda: self.field_oriented,
        #     # )
        #     DeferredCommand(
        #         lambda: self.drivetrain.set_speed_command(
        #             10000, self.drivetrain.old_rot
        #         ),
        #         self.drivetrain,
        #     )
        # ).onFalse(
        #     DeferredCommand(
        #         lambda: self.drivetrain.set_speed_command(
        #             # self.drivetrain.old_speed, self.drivetrain.old_rot
        #             12,
        #             Rotation2d.fromDegrees(180),
        #         ),
        #         self.drivetrain,
        #     )
        # )

        # self.driver_controller.button(button_lb).whileTrue(
        #     self.drivetrain.auto_rotate_joystick_drive(
        #         self.get_drive_x, self.get_drive_y, lambda: self.field_oriented
        #     )
        # )

        # def toggle_vision() -> ConditionalCommand:
        #     return ConditionalCommand(
        #         self.drivetrain.stop_vision(),
        #         self.drivetrain.start_vision(),
        #         lambda: self.drivetrain.using_vision,
        #     )

        # self.driver_controller.button(button_a).onTrue(toggle_vision())

        def set_invert():
            self.invert *= -1

        self.driver_controller.button(button_x).onTrue(
            DeferredCommand(lambda: InstantCommand(set_invert))
        )

        """operator controls"""
        Trigger(lambda: self.operator_controller.getThrottle() > 0.5).whileTrue(
            self.get_reef_score_command()
        ).onFalse(
            self.claw.stop()
            .andThen(self.fingers.score())
            .withTimeout(2)
            .andThen(self.fingers.stop())
        )

        Trigger(lambda: self.operator_controller.getRawAxis(trigger_lt) > 0.5).onTrue(
            intake_coral(self.elevator, self.wrist, self.claw)
        ).onFalse(self.get_intake_on_false())

        Trigger(lambda: abs(self.operator_controller.getRawAxis(5)) > 0.1).whileTrue(
            RepeatCommand(
                self.wrist.manual_control(
                    lambda: self.operator_controller.getRawAxis(5) / -5
                ),
            )
        ).onFalse(self.wrist.stop())

        # self.operator_controller.button(button_lpush).whileTrue(self.climber.reverse())
        # self.operator_controller.button(button_rpush).whileTrue(self.climber.climb())

        def increase_elevator_setpoint() -> None:
            self.level += 1
            if self.level > 3:
                self.level = 3

        def decrease_elevator_setpoint() -> None:
            self.level -= 1
            if self.level < 1:
                self.level = 1

        self.operator_controller.povUp().onTrue(
            WaitCommand(0.1).andThen(
                DeferredCommand(
                    lambda: InstantCommand(increase_elevator_setpoint),
                    self._fake_subsystem,
                )
            )
        )

        self.operator_controller.povDown().onTrue(
            WaitCommand(0.1).andThen(
                DeferredCommand(
                    lambda: InstantCommand(decrease_elevator_setpoint),
                    self._fake_subsystem,
                )
            )
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

        self.operator_controller.button(button_b).onTrue(self.get_stow())
        # self.operator_controller.button(button_b).onTrue(
        #     self.wrist.angle_zero()
        #     .andThen(self.claw.cage())
        #     .andThen(self.elevator.command_bottom())
        #     .andThen(self.wrist.angle_intake())
        #     .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        # )

        # self.operator_controller.button(button_rpush).whileTrue(
        #     self.wrist.angle_intake()
        # ).onFalse(self.wrist.stop())

        self.operator_controller.button(button_right).whileTrue(
            self.climber.climb()
        ).onFalse(self.climber.stop())
        self.operator_controller.button(button_left).whileTrue(
            self.climber.reverse()
        ).onFalse(self.climber.stop())

        self.operator_controller.button(button_rb).whileTrue(
            # self.wrist.angle_zero()
            self.claw.cage()
            # .andThen(self.elevator.command_intake())
            .andThen(self.wrist.angle_intake_slow())
        ).onFalse(self.wrist.stop().andThen(self.claw.coral()))

    def periodic(self) -> None:
        self.nettable.putNumber("Elevator Level", self.level)
        self.nettable.putBoolean("Coral", self.grabbing_coral)
        self.nettable.putNumber("Invert", self.invert)
        self.nettable.putBoolean("Field Oriented", self.field_oriented)

    def get_alliance(self) -> DriverStation.Alliance:
        return self.alliance

    def get_auto_command(self) -> Command:
        return commands2.cmd.none()
        # return self.auto_chooser.getSelected()
        # return blue_left_two_coral.get_auto(self.drivetrain)

    def get_auto_name(self) -> str:
        return ""
        # return self.auto_chooser.getSelected().getName()
