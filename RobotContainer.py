from commands2 import Command, WaitCommand, InstantCommand
from commands2.button import Trigger

from wpilib import Joystick, DriverStation

from subsystems.drivetrain import Drivetrain


class RobotContainer:
    def __init__(self) -> None:
        self.drivetrain = Drivetrain()
        self.driver_controller = Joystick(0)
        # TODO: set the bindings for the controller
        # this sets the motors to idle on disable
        self.driver_controller.setYChannel(1)
        self.driver_controller.setXChannel(0)
        self.driver_controller.setTwistChannel(4)
        self.driver_controller.setThrottleChannel(3)
        Trigger(DriverStation.isEnabled).onTrue(
            self.drivetrain.set_drive_idle(False)
        ).onTrue(self.drivetrain.set_turn_idle(False))
        Trigger(DriverStation.isEnabled).onFalse(
            self.drivetrain.set_drive_idle(True)
        ).onFalse(self.drivetrain.set_turn_idle(True))

    def set_teleop_bindings(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.drive_joystick(
                lambda: self.driver_controller.getX(),
                lambda: -self.driver_controller.getY(),
                lambda: -self.driver_controller.getTwist(),
                # this assumes that -1 is resting and 1 is full
                lambda: self.driver_controller.getThrottle() > 0,
            )
        )

    def unset_teleop_bindings(self) -> None:
        self.drivetrain.setDefaultCommand(WaitCommand(0))

    def get_auto_command(self) -> Command:
        return Command()
