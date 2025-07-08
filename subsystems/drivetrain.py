from subsystems.swerve_module import SwerveModule, ModuleLocation
from subsystems.vision import Vision

from typing import Callable

from commands2 import DeferredCommand, InstantCommand, Subsystem

from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    ChassisSpeeds,
    SwerveModuleState,
)
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.units import feetToMeters, degreesToRadians
import wpilib

from wpilib import Field2d, DriverStation, RobotBase, SmartDashboard

from ntcore import NetworkTableInstance
from ntcore.util import ntproperty

from navx import AHRS

# from pathplannerlib.auto import AutoBuilder, RobotConfig
# from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
# from pathplannerlib.logging import PathPlannerLogging


class Drivetrain(Subsystem):
    max_speed = ntproperty("000Drivetrain/max_speed", feetToMeters(6))
    max_angular_speed = ntproperty(
        "000Drivetrain/max_angular_speed", degreesToRadians(180)
    )

    def __init__(
        self,
        should_flip: Callable[[], bool] = lambda: DriverStation.getAlliance()
        == DriverStation.Alliance.kRed,
    ):
        super().__init__()

        self.fl = SwerveModule(ModuleLocation.FRONT_LEFT)
        self.fr = SwerveModule(ModuleLocation.FRONT_RIGHT)
        self.bl = SwerveModule(ModuleLocation.BACK_LEFT)
        self.br = SwerveModule(ModuleLocation.BACK_RIGHT)

        self.gyro = AHRS(AHRS.NavXComType.kUSB1)

        self.kinematics = SwerveDrive4Kinematics(
            self.fl.get_from_center(),
            self.fr.get_from_center(),
            self.bl.get_from_center(),
            self.br.get_from_center(),
        )

        self.vision = Vision()

        self.odometry = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),
            self.get_module_positions(),
            Pose2d(),
        )

        self.visionless_odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            self.get_module_positions(),
            Pose2d(),
        )

        self.should_flip = should_flip

        self.field = Field2d()
        self.visionless_field_pose = self.field.getObject("Visionless Pose")
        SmartDashboard.putData(self.field)
        SmartDashboard.putData(self)
        # SmartDashboard.putData(self.gyro)

        self.setpoint = ChassisSpeeds()

        self.nettable = NetworkTableInstance.getDefault().getTable("/000Drivetrain")

        self.swerve_pub = self.nettable.getStructArrayTopic(
            "SwerveStates", SwerveModuleState
        ).publish()

        self.pose_pub = self.nettable.getStructTopic("Pose", Pose2d).publish()
        self.setpoint_pub = self.nettable.getStructTopic(
            "Setpoint", ChassisSpeeds
        ).publish()

        self.speeds_pub = self.nettable.getStructTopic(
            "Actual Speeds", ChassisSpeeds
        ).publish()

        self.swerve_setpoint_pub = self.nettable.getStructArrayTopic(
            "Swerve Setpoints", SwerveModuleState
        ).publish()

        # robot_cfg = RobotConfig.fromGUISettings()
        # self.auto_builder = AutoBuilder.configure(
        #     self.get_pose,
        #     self.reset_pose,
        #     self.get_speeds,
        #     lambda speeds, _feedforward: self.run_chassis_speeds(
        #         speeds
        #         # ChassisSpeeds.fromRobotRelativeSpeeds(speeds, self.get_angle())
        #     ),
        #     (
        #         PPHolonomicDriveController(
        #             PIDConstants(0, 0, 0, 0), PIDConstants(2, 0, 0, 0)
        #         )
        #         if RobotBase.isReal()
        #         else PPHolonomicDriveController(
        #             PIDConstants(0.75), PIDConstants(2.0, 0, 0.0)
        #         )
        #     ),
        #     robot_cfg,
        #     self.should_flip,
        #     self,
        # )

        # SmartDashboard.putData(self.gyro)

    def periodic(self):
        self.run_chassis_speeds(self.setpoint)
        # self.odometry = self.vision.update_position(self.odometry)
        self.vision.update_position(self.odometry)
        new_pose = self.odometry.update(
            Rotation2d.fromDegrees(self.gyro.getAngle()),
            self.get_module_positions(),
        )
        self.visionless_field_pose.setPose(
            self.visionless_odometry.update(
                Rotation2d.fromDegrees(self.gyro.getAngle()),
                self.get_module_positions(),
            )
        )
        # self.odometry.addVisionMeasurement(
        #         Pose2d(), wpilib.Timer.getFPGATimestamp(),
        #     (5, 1, 10)
        # )
        self.field.setRobotPose(new_pose)
        self.swerve_pub.set(list(self.get_states()))
        self.pose_pub.set(new_pose)
        self.setpoint_pub.set(self.setpoint)
        self.speeds_pub.set(self.get_speeds())
        self.swerve_setpoint_pub.set(
            [self.fl.setpoint, self.fr.setpoint, self.bl.setpoint, self.br.setpoint]
        )
        # self.nettable.putString(
        #     "Running Command",
        #     (
        #         "None"
        #         if self.getCurrentCommand() is None
        #         else self.getCurrentCommand().getName()
        #     ),
        # )
        # SmartDashboard.putData("Drivetrain", self)
        return super().periodic()

    def simulationPeriodic(self):
        speeds = self.get_speeds()
        self.gyro.setAngleAdjustment(self.gyro.getAngle() + speeds.omega_dps * -0.02)
        _ = self.vision.sim_update(self.visionless_odometry.getPose())
        return super().simulationPeriodic()

    def get_module_positions(
        self,
    ) -> tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        return (
            self.fl.get_position(),
            self.fr.get_position(),
            self.bl.get_position(),
            self.br.get_position(),
        )

    def get_angle(self) -> Rotation2d:
        if self.should_flip():
            return (
                self.odometry.getEstimatedPosition().rotation()
                + Rotation2d.fromDegrees(180)
            )
            # return self.gyro.getRotation2d() + Rotation2d.fromDegrees(180)
        else:
            # return self.gyro.getRotation2d()
            return self.odometry.getEstimatedPosition().rotation()

    def get_states(
        self,
    ) -> tuple[
        SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
    ]:
        return (
            self.fl.get_state(),
            self.fr.get_state(),
            self.bl.get_state(),
            self.br.get_state(),
        )

    def get_speeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.get_states())

    def get_pose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()

    def stop(self) -> None:
        self.run_chassis_speeds(ChassisSpeeds())

    def stop_command(self) -> InstantCommand:
        return InstantCommand(self.stop)

    def run_chassis_speeds(self, speeds: ChassisSpeeds) -> None:
        # speeds = ChassisSpeeds.discretize(speeds, 0.02)
        fl, fr, bl, br = self.kinematics.toSwerveModuleStates(
            speeds, Translation2d(0, 0)
        )
        fl, fr, bl, br = self.kinematics.desaturateWheelSpeeds(
            (fl, fr, bl, br), self.fl.theoretial_max_vel
        )
        self.setpoint = speeds
        self.fl.set_state(fl)
        self.fr.set_state(fr)
        self.bl.set_state(bl)
        self.br.set_state(br)

    def run_percent(
        self, tx: float, ty: float, omega: float, field_relative: bool
    ) -> None:
        # self.nettable.putNumber("tx", tx)
        # self.nettable.putNumber("ty", ty)
        # self.nettable.putNumber("omega", omega)
        if field_relative:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                tx * self.max_speed,
                ty * self.max_speed,
                omega * self.max_angular_speed,
                self.get_angle(),
            )
        else:
            speeds = ChassisSpeeds(
                tx * self.max_speed, ty * self.max_speed, omega * self.max_speed
            )
        self.run_chassis_speeds(speeds)

    def reset_pose(self, new_pose: Pose2d) -> None:
        self.odometry.resetPose(new_pose)

    def reset_gyro(self, new_angle: Rotation2d) -> None:
        # if new_angle == Rotation2d() and not RobotBase.isSimulation():
        #     self.gyro.reset()
        # else:
        self.gyro.setAngleAdjustment(new_angle.degrees())

    def reset_gyro_command(self, new_angle: Rotation2d) -> DeferredCommand:
        return DeferredCommand(
            lambda: InstantCommand(lambda: self.reset_gyro(new_angle))
        )

    def set_drive_idle(self, coast: bool) -> None:
        self.fl.set_drive_idle(coast)
        self.fr.set_drive_idle(coast)
        self.bl.set_drive_idle(coast)
        self.br.set_drive_idle(coast)

    def set_turn_idle(self, coast: bool) -> None:
        self.fl.set_turn_idle(coast)
        self.fr.set_turn_idle(coast)
        self.bl.set_turn_idle(coast)
        self.br.set_turn_idle(coast)

    def set_drive_idle_command(self, coast: bool) -> InstantCommand:
        return InstantCommand(lambda: self.set_drive_idle(coast))

    def set_turn_idle_command(self, coast: bool) -> InstantCommand:
        return InstantCommand(lambda: self.set_turn_idle(coast))
