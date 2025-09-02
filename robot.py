from commands2 import Command, CommandScheduler
from ntcore import NetworkTableInstance
from wpilib import (
    DriverStation,
    RobotBase,
    SmartDashboard,
    TimedRobot,
    run,
    DataLogManager,
)
import wpilib

from wpimath.geometry import Pose3d, Translation3d, Rotation3d, Rotation2d
from wpimath.units import inchesToMeters


from RobotContainer import RobotContainer

from util import elastic

from subsystems.fingers import Fingers


class Robot(TimedRobot):
    m_autonomousCommand: Command
    m_robotContainer: RobotContainer
    curr_auto: str = ""
    prev_auto: str = ""

    # Initialize Robot
    def robotInit(self):
        self.m_robotContainer = RobotContainer()
        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())

    def robotPeriodic(self) -> None:
        try:
            CommandScheduler.getInstance().run()
        except Exception as e:
            wpilib.reportError(f"Got Error from Command Scheduler: {e}", True)
        # if self.m_robotContainer:
        #     self.m_robotContainer.periodic()

    def autonomousInit(self):
        elastic.select_tab("Autonomous")
        if self.m_robotContainer is not None:
            self.m_autonomousCommand = self.m_robotContainer.get_auto_command()

            if self.m_autonomousCommand is not None:
                # self.m_autonomousCommand.schedule()
                CommandScheduler.getInstance().schedule(self.m_autonomousCommand)

    def autonomousPeriodic(self):
        # if self.m_autonomousCommand is not None:
        #     if not self.m_autonomousCommand.isScheduled():
        #         self.m_autonomousCommand.schedule()
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
        #     # self.m_robotContainer.wrist.angle_zero().schedule()
        #     self.m_robotContainer.elevator.stop().schedule()
        #     self.m_robotContainer.fingers.stop().schedule()
        #     # self.m_robotContainer.drivetrain.stop_command().schedule()

    def teleopPeriodic(self):
        pass

    def teleopExit(self):
        pass

    # Test Robot Functions
    def testInit(self) -> None:
        pass

    # def testInit(self):
    #     if RobotBase.isSimulation() and self.m_robotContainer is not None:
    #         if (
    #             self.m_robotContainer.elevator is not None
    #             and self.m_robotContainer.wrist is not None
    #             and self.m_robotContainer.claw is not None
    #         ):
    #             self.m_robotContainer.wrist.angle_zero().andThen(
    #                 self.m_robotContainer.claw.cage()
    #             ).andThen(self.m_robotContainer.elevator.command_bottom()).andThen(
    #                 self.m_robotContainer.wrist.angle_intake()
    #             ).withInterruptBehavior(
    #                 Command.InterruptionBehavior.kCancelSelf
    #             ).schedule()
    #     pass

    def testPeriodic(self):
        pass

    def testExit(self):
        pass

    # Disabled Robot Functions
    def disabledInit(self):
        pass

    def disabledPeriodic(self) -> None:
        pass

    def disabledExit(self):
        pass

    def _simulationInit(self) -> None:
        self.nettable = NetworkTableInstance.getDefault().getTable("Mechanism3dPoses")
        self.zero_posepub = self.nettable.getStructArrayTopic(
            "ZeroPoses", Pose3d
        ).publish()
        self.final_posepub = self.nettable.getStructArrayTopic(
            "FinalPoses", Pose3d
        ).publish()
        return super()._simulationInit()

    def _simulationPeriodic(self) -> None:
        self.zero_posepub.set([a := Pose3d(5, 5, 5, Rotation3d()), a, a, Pose3d()])
        if self.m_robotContainer is not None:
            if hasattr(self.m_robotContainer, "elevator"):
                ele_pose = Translation3d(
                    0.245,
                    0,
                    self.m_robotContainer.elevator.get_height() + inchesToMeters(9.384),
                )
                wrist_pose = ele_pose + Translation3d(0, 0, 0.1924304)
                wrist_angle = Rotation3d(
                    0,
                    -self.m_robotContainer.wrist.get_angle().radians(),
                    0,
                )
                self.final_posepub.set(
                    [
                        Pose3d(
                            ele_pose,
                            Rotation3d(
                                0,
                                0,
                                0,
                            ),
                        ),
                        (
                            Pose3d(
                                wrist_pose,
                                wrist_angle,
                            )
                            if hasattr(self.m_robotContainer, "wrist")
                            else None
                        ),
                        (
                            Pose3d(
                                wrist_pose
                                + Translation3d(
                                    0,
                                    self.m_robotContainer.claw.get_distance() / 2,
                                    0,
                                ),
                                wrist_angle,
                            )
                            if hasattr(self.m_robotContainer, "claw")
                            else None
                        ),
                        (
                            Pose3d(
                                wrist_pose
                                - Translation3d(
                                    0,
                                    self.m_robotContainer.claw.get_distance() / 2,
                                    0,
                                ),
                                wrist_angle,
                            )
                            if hasattr(self.m_robotContainer, "claw")
                            else None
                        ),
                    ]
                )
        return super()._simulationPeriodic()


# Start the Robot when Executing Code
if __name__ == "__main__":
    run(Robot)
