from dataclasses import field
from math import pi
import time

import commands2

from subsystems.swerve_module import SwerveModule
from subsystems.navx_gryo import NavX
from subsystems.sim_gyro import SimGyro

from auto import positions

from commands2 import (
    DeferredCommand,
    ParallelRaceGroup,
    SequentialCommandGroup,
    StartEndCommand,
    Subsystem,
    InstantCommand,
    RunCommand,
    WaitCommand,
    WrapperCommand,
)

import wpilib
from wpilib import Field2d, RobotBase, DriverStation, SmartDashboard
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from wpimath.units import inchesToMeters, feetToMeters, metersToFeet, feet
from wpimath.controller import (
    ProfiledPIDController,
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.trajectory import TrapezoidProfile, Trajectory, TrapezoidProfileRadians
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    ChassisSpeeds,
    SwerveModuleState,
)

from wpimath.estimator import SwerveDrive4PoseEstimator

from wpimath import applyDeadband

from ntcore import NetworkTable, NetworkTableInstance, EventFlags, ValueEventData, Event

import typing

from subsystems.vision import Vision


class Drivetrain(Subsystem):
    def __init__(
        self,
        get_alliance: typing.Callable[[], DriverStation.Alliance],
        constant_of_acceleration: float = 15,
    ):
        self.get_alliance = get_alliance
        self.alliance = get_alliance()

        """member instantiation"""
        self.constant_of_acceleration = constant_of_acceleration
        self.max_velocity_mps = feetToMeters(12)
        if RobotBase.isReal():
            self.max_angular_velocity = Rotation2d.fromDegrees(180)
            # self.max_angular_velocity = Rotation2d.fromDegrees(90)
        else:
            self.max_angular_velocity = Rotation2d(0)

        max_accel = self.max_velocity_mps * constant_of_acceleration

        self.fl = SwerveModule(
            "fl", 6, 8, 7, False, True, self.max_velocity_mps, max_accel * 4
        )
        self.fr = SwerveModule(
            "fr", 15, 17, 16, False, True, self.max_velocity_mps, max_accel * 4
        )
        self.bl = SwerveModule(
            "bl", 9, 11, 10, False, True, self.max_velocity_mps, max_accel * 4
        )
        self.br = SwerveModule(
            "br", 12, 14, 13, False, True, self.max_velocity_mps, max_accel * 4
        )

        self.x_pid = ProfiledPIDController(
            0.6,
            0,
            0.0,
            TrapezoidProfile.Constraints(self.max_velocity_mps, max_accel),
        )

        self.y_pid = ProfiledPIDController(
            0.6,
            0,
            0.0,
            TrapezoidProfile.Constraints(self.max_velocity_mps, max_accel),
        )

        self.t_pid = ProfiledPIDControllerRadians(
            0.6,
            0,
            0.1,
            TrapezoidProfileRadians.Constraints(
                self.max_angular_velocity.degrees(),
                self.max_angular_velocity.degrees() * constant_of_acceleration,
            ),
        )

        self.t_pid.enableContinuousInput(-pi, pi)
        self.t_pid.setIntegratorRange(0, 0.25)
        self.t_pid.setIZone(pi / 2)

        self.x_pid.setTolerance(0.25)
        self.y_pid.setTolerance(0.25)
        self.t_pid.setTolerance(pi / 8)

        if RobotBase.isReal():
            # self.gyro = NavX.fromMXP()
            self.gyro = NavX.fromUSB(1)
        else:
            self.gyro = SimGyro()

        """nettables"""
        self.nettable = NetworkTableInstance.getDefault().getTable("000Drivetrain")

        def nettable_listener(_nt: NetworkTable, key: str, ev: Event):
            if isinstance(v := ev.data, ValueEventData):
                if key == "config/max_velocity_fps":
                    self.max_velocity_mps = feetToMeters(v.value.value())

                    self.x_pid.setConstraints(
                        TrapezoidProfile.Constraints(
                            self.max_velocity_mps,
                            self.max_velocity_mps * self.constant_of_acceleration,
                        )
                    )

                    self.y_pid.setConstraints(
                        TrapezoidProfile.Constraints(
                            self.max_velocity_mps,
                            self.max_velocity_mps * self.constant_of_acceleration,
                        )
                    )

                    self.fl.set_max_vel(self.max_velocity_mps)
                    self.fr.set_max_vel(self.max_velocity_mps)
                    self.bl.set_max_vel(self.max_velocity_mps)
                    self.br.set_max_vel(self.max_velocity_mps)
                elif key == "config/max_angular_velocity_degps":
                    self.max_angular_velocity = Rotation2d.fromDegrees(v.value.value())
                    self.t_pid.setConstraints(
                        TrapezoidProfileRadians.Constraints(
                            self.max_angular_velocity.degrees(),
                            self.max_angular_velocity.degrees()
                            * self.constant_of_acceleration,
                        )
                    )

                elif key[1:].startswith("PID"):
                    const = key.split("/")[1]
                    pid = key[0]
                    if pid == "x":
                        if const == "P":
                            self.x_pid.setP(v.value.value())
                        elif const == "I":
                            self.x_pid.setI(v.value.value())
                        elif const == "D":
                            self.x_pid.setD(v.value.value())

                    elif pid == "y":
                        if const == "P":
                            self.y_pid.setP(v.value.value())
                        elif const == "I":
                            self.y_pid.setI(v.value.value())
                        elif const == "D":
                            self.y_pid.setD(v.value.value())

                    elif pid == "t":
                        if const == "P":
                            self.t_pid.setP(v.value.value())
                        elif const == "I":
                            self.t_pid.setI(v.value.value())
                        elif const == "D":
                            self.t_pid.setD(v.value.value())

        self.nettable.putNumber(
            "config/max_velocity_fps", metersToFeet(self.max_velocity_mps)
        )
        self.nettable.putNumber(
            "config/max_angular_velocity_degps", self.max_angular_velocity.degrees()
        )
        self.nettable.addListener(
            "config/max_velocity_fps", EventFlags.kValueAll, nettable_listener
        )

        self.nettable.addListener(
            "config/max_angular_velocity_degps", EventFlags.kValueAll, nettable_listener
        )

        self.nettable.addListener("xPID/P", EventFlags.kValueAll, nettable_listener)
        self.nettable.addListener("xPID/I", EventFlags.kValueAll, nettable_listener)
        self.nettable.addListener("xPID/D", EventFlags.kValueAll, nettable_listener)

        self.nettable.addListener("yPID/P", EventFlags.kValueAll, nettable_listener)
        self.nettable.addListener("yPID/I", EventFlags.kValueAll, nettable_listener)
        self.nettable.addListener("yPID/D", EventFlags.kValueAll, nettable_listener)

        self.nettable.addListener("tPID/P", EventFlags.kValueAll, nettable_listener)
        self.nettable.addListener("tPID/I", EventFlags.kValueAll, nettable_listener)
        self.nettable.addListener("tPID/D", EventFlags.kValueAll, nettable_listener)

        self.nettable.putNumber("xPID/P", self.x_pid.getP())
        self.nettable.putNumber("xPID/I", self.x_pid.getI())
        self.nettable.putNumber("xPID/D", self.x_pid.getD())

        self.nettable.putNumber("yPID/P", self.y_pid.getP())
        self.nettable.putNumber("yPID/I", self.y_pid.getI())
        self.nettable.putNumber("yPID/D", self.y_pid.getD())

        self.nettable.putNumber("tPID/P", self.t_pid.getP())
        self.nettable.putNumber("tPID/I", self.t_pid.getI())
        self.nettable.putNumber("tPID/D", self.t_pid.getD())

        self.field = Field2d()
        # these should be changed when actually using the field
        self.field.setRobotPose(Pose2d(0, 0, 0))

        """kinematics"""
        # module locations
        fl_position = Translation2d(inchesToMeters(11.25), inchesToMeters(11.25))
        fr_position = Translation2d(inchesToMeters(11.25), -inchesToMeters(11.25))
        bl_position = Translation2d(-inchesToMeters(11.25), inchesToMeters(11.25))
        br_position = Translation2d(-inchesToMeters(11.25), -inchesToMeters(11.25))

        # kinematics - turns robot speeds into individual states for each module
        self.kinematics = SwerveDrive4Kinematics(
            fl_position, fr_position, bl_position, br_position
        )

        """odometry"""
        self.odometry = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.get_angle(),
            (
                self.fl.get_position(),
                self.fr.get_position(),
                self.bl.get_position(),
                self.br.get_position(),
            ),
            Pose2d.fromFeet(0, 0, Rotation2d.fromDegrees(0)),
        )

        self.vision = Vision()

        self.is_real = RobotBase.isReal()

        self.old_speed = self.max_velocity_mps
        self.old_rot = self.max_angular_velocity

    def periodic(self) -> None:
        if (a := self.get_alliance()) != self.alliance:
            self.alliance = a
            self.nettable.putString(
                "Alliance",
                (
                    "Blue"
                    if self.alliance == DriverStation.Alliance.kBlue
                    else (
                        "Red"
                        if self.alliance == DriverStation.Alliance.kRed
                        else "None"
                    )
                ),
            )

        self.odometry = self.vision.update_position(self.odometry)

        position = self.odometry.update(
            self.gyro.get_angle(),
            (
                self.fl.get_position(),
                self.fr.get_position(),
                self.bl.get_position(),
                self.br.get_position(),
            ),
        )

        curr_speed = self.kinematics.toChassisSpeeds(
            (
                self.fl.get_state(),
                self.fr.get_state(),
                self.bl.get_state(),
                self.br.get_state(),
            )
        )

        self.nettable.putNumber("velocity/vx (fps)", curr_speed.vx_fps)
        self.nettable.putNumber("velocity/vy (fps)", curr_speed.vy_fps)
        self.nettable.putNumber("velocity/omega (degps)", curr_speed.omega_dps)

        self.nettable.putNumber("state/x (ft)", position.x_feet)
        self.nettable.putNumber("state/y (ft)", position.y_feet)
        self.nettable.putNumber("state/theta (deg)", self.get_angle().degrees())

        self.nettable.putNumber("tPID/error (deg)", self.t_pid.getPositionError())

        if c := self.getCurrentCommand():
            self.nettable.putString("Running Command", c.getName())
        else:
            self.nettable.putString("Running Command", "None")

        self.field.setRobotPose(position)
        SmartDashboard.putData(self.field)

    def simulationPeriodic(self) -> None:
        curr_vel = self.kinematics.toChassisSpeeds(
            (
                self.fl.get_state(),
                self.fr.get_state(),
                self.bl.get_state(),
                self.br.get_state(),
            )
        )

        curr_pose = self.get_pose()
        self.odometry.resetPose(
            Pose2d(
                curr_pose.x + curr_vel.vx / 50,
                curr_pose.y + curr_vel.vy / 50,
                curr_pose.rotation() + Rotation2d(curr_vel.omega) / 50,
            )
        )

        self.field.setRobotPose(self.get_pose())
        SmartDashboard.putData(self.field)
        return super().simulationPeriodic()

    """getters"""

    def get_kinematics(self) -> SwerveDrive4Kinematics:
        return self.kinematics

    def get_odometry(self) -> SwerveDrive4PoseEstimator:
        return self.odometry

    def get_pose(self) -> Pose2d:
        # mayble flip if on red. Ideally just based on starting position, though
        return self.odometry.getEstimatedPosition()

    def get_angle(self) -> Rotation2d:
        if RobotBase.isReal():
            return self.get_pose().rotation()
        else:
            return self.gyro.get_angle()

    def get_module_positions(
        self,
    ) -> typing.Tuple[
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

    def get_module_states(
        self,
    ) -> typing.Tuple[
        SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
    ]:
        return (
            self.fl.get_state(),
            self.fr.get_state(),
            self.bl.get_state(),
            self.br.get_state(),
        )

    def get_speeds(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.get_module_states())

    """setters"""

    def reset_gyro(
        self, new_angle: Rotation2d = Rotation2d.fromDegrees(0)
    ) -> InstantCommand:
        return InstantCommand(lambda: self.gyro.reset(new_angle))

    def reset_pose(self, pose: Pose2d) -> SequentialCommandGroup:
        return InstantCommand(
            lambda: self.odometry.resetPosition(
                self.gyro.get_angle(), self.get_module_positions(), pose
            ),
            self,
        )

    def _set_drive_idle(self, coast: bool) -> None:
        self.fl.set_drive_idle(coast)
        self.fr.set_drive_idle(coast)
        self.bl.set_drive_idle(coast)
        self.br.set_drive_idle(coast)

    def set_drive_idle(self, coast: bool) -> InstantCommand:
        return InstantCommand(lambda: self._set_drive_idle(coast), self)

    def _set_turn_idle(self, coast: bool) -> None:
        self.fl.set_turn_idle(coast)
        self.fr.set_turn_idle(coast)
        self.bl.set_turn_idle(coast)
        self.br.set_turn_idle(coast)

    def set_turn_idle(self, coast: bool) -> InstantCommand:
        return InstantCommand(lambda: self._set_turn_idle(coast), self)

    def _run_chassis_speeds(
        self,
        speeds: ChassisSpeeds,
        center_of_rotation: Translation2d = Translation2d(0, 0),
    ) -> None:
        states = self.kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(speeds, 0.02), center_of_rotation
        )
        self.nettable.putNumber("commandedXVel fps", speeds.vx_fps)
        self.nettable.putNumber("commandedYVel fps", speeds.vy_fps)
        self.nettable.putNumber("commandedThetaVel degps", speeds.omega_dps)

        self._run_module_states(states)

    def _run_module_states(
        self,
        states: tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ],
    ) -> None:
        states = self.kinematics.desaturateWheelSpeeds(states, self.max_velocity_mps)
        self.fl.set_state(states[0])
        self.fr.set_state(states[1])
        self.bl.set_state(states[2])
        self.br.set_state(states[3])

    def stop(self) -> InstantCommand:
        return InstantCommand(lambda: self._run_chassis_speeds(ChassisSpeeds()), self)

    def drive_joystick(
        self,
        get_x: typing.Callable[[], float],
        get_y: typing.Callable[[], float],
        get_theta: typing.Callable[[], float],
        use_field_oriented: typing.Callable[[], bool],
    ) -> WrapperCommand:
        if not self.is_real:
            return RunCommand(
                lambda: self._run_chassis_speeds(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        applyDeadband(get_x(), 0.1, 1.0) * self.max_velocity_mps,
                        applyDeadband(get_y(), 0.1, 1.0) * self.max_velocity_mps,
                        applyDeadband(get_theta(), 0.1, 1.0)
                        * self.max_angular_velocity.radians(),
                        -self.get_angle(),
                    )
                    if use_field_oriented()
                    else ChassisSpeeds.fromFieldRelativeSpeeds(
                        applyDeadband(get_x(), 0.1, 1.0) * self.max_velocity_mps,
                        applyDeadband(get_y(), 0.1, 1.0) * self.max_velocity_mps,
                        applyDeadband(get_theta(), 0.1, 1.0)
                        * self.max_angular_velocity.radians(),
                        self.get_angle(),
                    )
                ),
                self,
            ).withName("Drive Joystick")

        return RunCommand(
            lambda: self._run_chassis_speeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    applyDeadband(get_x(), 0.1, 1.0) * self.max_velocity_mps,
                    applyDeadband(get_y(), 0.1, 1.0) * self.max_velocity_mps,
                    applyDeadband(get_theta(), 0.1, 1.0)
                    * self.max_angular_velocity.radians(),
                    self.get_angle(),
                )
                if use_field_oriented()
                else ChassisSpeeds(
                    applyDeadband(get_x(), 0.1, 1.0) * self.max_velocity_mps,
                    applyDeadband(get_y(), 0.1, 1.0) * self.max_velocity_mps,
                    applyDeadband(get_theta(), 0.1, 1.0)
                    * self.max_angular_velocity.radians(),
                )
            ),
            self,
        ).withName("Drive Joystick")

    def drive_position(
        self,
        position: Pose2d,
    ) -> ParallelRaceGroup | WrapperCommand:
        return (
            self.drive_joystick(
                lambda: self.x_pid.calculate(
                    self.get_pose().X(),
                    position.X(),
                ),
                lambda: self.y_pid.calculate(
                    self.get_pose().Y(),
                    position.Y(),
                ),
                lambda: (
                    self.t_pid.calculate(
                        self.get_angle().radians(),
                        position.rotation().radians(),
                    )
                ),
                lambda: True,
            )
            .withName(f"Drive Position")
            .onlyWhile(
                lambda: (
                    abs(self.x_pid.getPositionError()) > feetToMeters(0.5)
                    or (abs(self.y_pid.getPositionError()) > feetToMeters(0.5))
                    or (
                        (abs(self.t_pid.getPositionError()) > pi / 16)
                        if self.is_real
                        else False
                    )
                    or abs((v := self.get_speeds()).vx) > feetToMeters(5)
                    or abs(v.vy) > feetToMeters(5)
                    or (abs(v.omega_dps) > 15 if self.is_real else False)
                    or abs(self.x_pid.getSetpoint().position - position.X()) > 0.1
                    or abs(self.y_pid.getSetpoint().position - position.Y()) > 0.1
                )
            )
        )

    def get_closest(self, location_type: str) -> Pose2d:
        if location_type == "all":
            return self.get_pose().nearest(
                [
                    self.get_closest("reef"),
                    self.get_closest("algae"),
                    self.get_closest("intake"),
                ]
            )
        elif location_type == "reef":
            if self.alliance == DriverStation.Alliance.kBlue:
                drive_positions = [
                    positions.blue_reef_a,
                    positions.blue_reef_b,
                    positions.blue_reef_c,
                    positions.blue_reef_d,
                    positions.blue_reef_e,
                    positions.blue_reef_f,
                    positions.blue_reef_g,
                    positions.blue_reef_h,
                    positions.blue_reef_i,
                    positions.blue_reef_j,
                    positions.blue_reef_k,
                    positions.blue_reef_l,
                ]
            else:
                drive_positions = [
                    positions.red_reef_a,
                    positions.red_reef_b,
                    positions.red_reef_c,
                    positions.red_reef_d,
                    positions.red_reef_e,
                    positions.red_reef_f,
                    positions.red_reef_g,
                    positions.red_reef_h,
                    positions.red_reef_i,
                    positions.red_reef_j,
                    positions.red_reef_k,
                    positions.red_reef_l,
                ]

        elif location_type == "algae":
            if self.alliance == DriverStation.Alliance.kBlue:
                drive_positions = [
                    positions.blue_algae_ab,
                    positions.blue_algae_cd,
                    positions.blue_algae_ef,
                    positions.blue_algae_gh,
                    positions.blue_algae_ij,
                    positions.blue_algae_kl,
                ]
            else:
                drive_positions = [
                    positions.red_algae_ab,
                    positions.red_algae_cd,
                    positions.red_algae_ef,
                    positions.red_algae_gh,
                    positions.red_algae_ij,
                    positions.red_algae_kl,
                ]

        elif location_type == "intake":
            if self.alliance == DriverStation.Alliance.kBlue:
                drive_positions = [
                    positions.blue_coral_intake_left_center,
                    positions.blue_coral_intake_right_center,
                ]
            else:
                drive_positions = [
                    positions.red_coral_intake_left_center,
                    positions.red_coral_intake_right_center,
                ]
        else:
            wpilib.reportWarning(
                f"The Drive Closest got a bad location type {location_type=}"
            )
            drive_positions = [self.get_pose()]
        return self.get_pose().nearest(drive_positions)

    def drive_closest_reef(self) -> WrapperCommand:
        return DeferredCommand(
            lambda: self.drive_position(self.get_closest("reef"))
        ).withName("Drive Closest Reef")

    def drive_closest_algae(self) -> WrapperCommand:
        return DeferredCommand(
            lambda: self.drive_position(self.get_closest("algae"))
        ).withName("Drive Closest Algae")

    def drive_near_coral_station(self) -> WrapperCommand:
        return DeferredCommand(
            lambda: self.drive_position(self.get_closest("intake"))
        ).withName("Drive Closest Intake")

    def get_closest_algae_level(self) -> int:
        if self.alliance == DriverStation.Alliance.kBlue:
            return (
                2
                if self.get_pose().nearest(
                    [
                        positions.blue_algae_ab,
                        positions.blue_algae_cd,
                        positions.blue_algae_ef,
                        positions.blue_algae_gh,
                        positions.blue_algae_ij,
                        positions.blue_algae_kl,
                    ]
                )
                in [
                    positions.blue_algae_cd,
                    positions.blue_algae_gh,
                    positions.blue_algae_kl,
                ]
                else 3
            )

        return (
            2
            if self.get_pose().nearest(
                [
                    positions.red_algae_ab,
                    positions.red_algae_cd,
                    positions.red_algae_ef,
                    positions.red_algae_gh,
                    positions.red_algae_ij,
                    positions.red_algae_kl,
                ]
            )
            in [
                positions.red_algae_cd,
                positions.red_algae_gh,
                positions.red_algae_kl,
            ]
            else 3
        )

    def auto_rotate_joystick_drive(
        self,
        x: typing.Callable[[], float],
        y: typing.Callable[[], float],
        field_oriented: typing.Callable[[], bool],
    ) -> WrapperCommand:
        return self.drive_joystick(
            x,
            y,
            lambda: self.t_pid.calculate(
                self.get_angle().radians(), self.get_closest("all").rotation().radians()
            ),
            field_oriented,
        )

    def set_speed(self, drive_speed_mps: float, turn_speed: Rotation2d) -> None:
        self.max_velocity_mps = drive_speed_mps
        if RobotBase.isReal():
            self.max_angular_velocity = turn_speed
        else:
            self.max_angular_velocity = Rotation2d(0)
        self.x_pid.setConstraints(
            TrapezoidProfile.Constraints(
                self.max_velocity_mps,
                self.max_velocity_mps * self.constant_of_acceleration,
            )
        )
        self.y_pid.setConstraints(
            TrapezoidProfile.Constraints(
                self.max_velocity_mps,
                self.max_velocity_mps * self.constant_of_acceleration,
            )
        )
        self.t_pid.setConstraints(
            TrapezoidProfileRadians.Constraints(
                self.max_angular_velocity.radians(),
                self.max_angular_velocity.radians() * self.constant_of_acceleration,
            )
        )

    def set_speed_command(
        self, max_speed_mps: float, max_angular_speed: Rotation2d
    ) -> InstantCommand:
        def set_old_speeds():
            self.old_speed = self.max_velocity_mps
            self.old_rot = self.max_angular_velocity

        return InstantCommand(lambda: self.set_speed(max_speed_mps, max_angular_speed))
