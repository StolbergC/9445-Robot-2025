from commands2 import Command, CommandScheduler
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard, TimedRobot, Watchdog, run, DataLogManager

from RobotContainer import RobotContainer, button_lb
from subsystems.climber import Climber

from util import elastic


class Robot(TimedRobot):
    m_autonomousCommand: Command | None = None
    m_robotContainer: RobotContainer | None = None

    # Initialize Robot
    def robotInit(self):
        self.m_robotContainer = RobotContainer()
        DataLogManager.start()

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()
        if self.m_robotContainer:
            self.m_robotContainer.periodic()

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
        # if self.m_robotContainer:
        # self.m_robotContainer.drivetrain.stop_command().schedule()

    # Teleop Robot Functions
    def teleopInit(self):
        elastic.select_tab("Teleoperated")
        if self.m_robotContainer is not None:
            self.m_robotContainer.set_teleop_bindings()
            # self.m_robotContainer.wrist.angle_zero().schedule()
            self.m_robotContainer.elevator.stop().schedule()
            self.m_robotContainer.fingers.stop().schedule()
            # self.m_robotContainer.drivetrain.stop_command().schedule()

    def teleopPeriodic(self):
        pass

    def teleopExit(self):
        pass

    # Test Robot Functions
    def testInit(self):
        self.nettable = NetworkTableInstance.getDefault().getTable("0000Pit Rest")
        elastic.select_tab("Test")
        if self.m_robotContainer is not None:
            self.nettable.putBoolean("Zero Claw", False)
            self.nettable.putBoolean("Return Wrist", False)
            self.nettable.putBoolean("Home Elevator", False)

    def testPeriodic(self):
        if self.m_robotContainer:
            if self.nettable.getBoolean("Zero Claw", False):
                self.m_robotContainer.wrist.angle_zero().andThen(
                    self.m_robotContainer.claw.cage()
                ).schedule()
                # self.nettable.putBoolean("Zero Claw", False)
            if self.nettable.getBoolean("Zero Claw", True) == False:
                self.m_robotContainer.claw.stop().schedule()
            if self.nettable.getBoolean("Return Wrist", False):
                self.m_robotContainer.wrist.angle_full_up().schedule()
            if self.nettable.getBoolean("Return Wrist", True) == False:
                self.m_robotContainer.wrist.stop().schedule()
            if self.nettable.getBoolean("Home Elevator", True) == False:
                self.m_robotContainer.elevator.stop().schedule()

    def testExit(self):
        pass

    # Disabled Robot Functions
    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        pass

    def disabledExit(self):
        pass

    # def SimulationPeriodic(self) -> None:
    #     CommandScheduler.getInstance().run()


# Start the Robot when Executing Code
if __name__ == "__main__":
    run(Robot)
