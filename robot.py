from commands2 import Command, CommandScheduler
from wpilib import SmartDashboard, TimedRobot, Watchdog, run

from RobotContainer import RobotContainer, button_lb
from subsystems.climber import Climber

from util import elastic


class Robot(TimedRobot):
    m_autonomousCommand: Command | None = None
    m_robotContainer: RobotContainer | None = None

    # Initialize Robot
    def robotInit(self):
        self.m_robotContainer = RobotContainer()
        # Watchdog(0.05, lambda: None).disable()  # .suppressTimeoutMessage(True)

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    # Autonomous Robot Functions
    def autonomousInit(self):
        elastic.select_tab("Autonomous")
        if self.m_robotContainer is not None:
            self.m_autonomousCommand = self.m_robotContainer.get_auto_command()

            if self.m_autonomousCommand is not None:
                # self.m_autonomousCommand.schedule()
                CommandScheduler.getInstance().schedule(self.m_autonomousCommand)

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        if self.m_autonomousCommand:
            self.m_autonomousCommand.cancel()

    # Teleop Robot Functions
    def teleopInit(self):
        elastic.select_tab("Teleoperated")
        if self.m_robotContainer is not None:
            self.m_robotContainer.set_teleop_bindings()

    def teleopPeriodic(self):
        pass

    def teleopExit(self):
        pass

    # Test Robot Functions
    def testInit(self):
        elastic.select_tab("Test")
        # self.m_robotContainer.operator_controller.button(button_lb).whileTrue(
        # self.m_robotContainer.climber.reverse()
        # )
        ...

    def testPeriodic(self):
        pass

    def testExit(self):
        pass

    # Disabled Robot Functions
    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass

    def disabledExit(self):
        pass

    def SimulationPeriodic(self) -> None:
        CommandScheduler.getInstance().run()


# Start the Robot when Executing Code
if __name__ == "__main__":
    run(Robot)
