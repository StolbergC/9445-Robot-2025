from commands2 import Command, CommandScheduler
from wpilib import TimedRobot, Watchdog, run

from RobotContainer import RobotContainer


class Robot(TimedRobot):
    m_autonomousCommand: Command = None
    m_robotContainer: RobotContainer = None

    # Initialize Robot
    def robotInit(self):
        self.m_robotContainer = RobotContainer()
        Watchdog(0.05, lambda: None).suppressTimeoutMessage(False)

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()
        # self.m_robotContainer.interface.periodic()

    # Autonomous Robot Functions
    def autonomousInit(self):
        self.m_autonomousCommand = self.m_robotContainer.get_auto_command()

        if self.m_autonomousCommand is not None:
            self.m_autonomousCommand.schedule()

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        # self.m_autonomousCommand.cancel()
        ...

    # Teleop Robot Functions
    def teleopInit(self):
        self.m_robotContainer.set_teleop_bindings()

    def teleopPeriodic(self):
        pass

    def teleopExit(self):
        pass

    # Test Robot Functions
    def testInit(self):
        pass

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
