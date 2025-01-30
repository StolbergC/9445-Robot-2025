from math import pi

from subsystems.swerve_module import SwerveModule
from subsystems.navx_gryo import NavX
from subsystems.sim_gyro import SimGyro

from commands2 import (
    SequentialCommandGroup,
    StartEndCommand,
    Subsystem,
    InstantCommand,
    RunCommand,
    WrapperCommand,
)

from wpilib import Field2d, RobotBase, reportWarning
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from wpimath.units import inchesToMeters, feetToMeters, metersToFeet, feet
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
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
    def __init__(self):
        """member instantiation"""
        self.max_velocity_mps = feetToMeters(10)
        self.max_angular_velocity = Rotation2d.fromDegrees(180)

        max_accel = self.max_velocity_mps * 4

        self.fl = SwerveModule(
            "fl", 6, 8, 7, False, True, self.max_velocity_mps, max_accel
        )
        self.fr = SwerveModule(
            "fr", 15, 17, 16, False, True, self.max_velocity_mps, max_accel
        )
        self.bl = SwerveModule(
            "bl", 9, 11, 10, False, True, self.max_velocity_mps, max_accel
        )
        self.br = SwerveModule(
            "br", 12, 14, 13, False, True, self.max_velocity_mps, max_accel
        )

        self.x_pid = ProfiledPIDController(
            1.5,
            0,
            0,
            TrapezoidProfile.Constraints(
                self.max_velocity_mps, self.max_velocity_mps * 5
            ),
        )

        self.y_pid = ProfiledPIDController(
            1.5,
            0,
            0,
            TrapezoidProfile.Constraints(
                self.max_velocity_mps, self.max_velocity_mps * 5
            ),
        )

        self.t_pid = ProfiledPIDController(
            1.5,
            0,
            0.1,
            TrapezoidProfile.Constraints(
                self.max_angular_velocity.degrees(),
                self.max_angular_velocity.degrees() * 5,
            ),
        )

        self.t_pid.enableContinuousInput(-pi, pi)
        self.t_pid.setIntegratorRange(0, 0.25)
        self.t_pid.setIZone(pi / 2)

        if RobotBase.isReal():
            # self.gyro = NavX.fromMXP()
            self.gyro = NavX.fromUSB(1)
        else:
            self.gyro = SimGyro()

        """nettables"""
        self.nettable = NetworkTableInstance.getDefault().getTable("000Drivetrain")
        self.nettable.putNumber(
            "config/max_velocity_fps", metersToFeet(self.max_velocity_mps)
        )
        self.nettable.putNumber(
            "config/max_angular_velocity_degps", self.max_angular_velocity.degrees()
        )

        def nettable_listener(_nt: NetworkTable, key: str, ev: Event):
            self.nettable.putString("Trying To Update", key)
            if isinstance(v := ev.data, ValueEventData):
                if key == "max_velocity_fps":
                    self.max_velocity_mps = feetToMeters(v.value.value())
                    self.x_pid.setConstraints(
                        TrapezoidProfile.Constraints(
                            self.max_velocity_mps, self.max_velocity_mps * 5
                        )
                    )
                    self.y_pid.setConstraints(
                        TrapezoidProfile.Constraints(
                            self.max_velocity_mps, self.max_velocity_mps * 5
                        )
                    )
                    self.fl.set_max_vel(self.max_velocity_mps)
                    self.fr.set_max_vel(self.max_velocity_mps)
                    self.bl.set_max_vel(self.max_velocity_mps)
                    self.br.set_max_vel(self.max_velocity_mps)
                elif key == "max_angular_velocity_degps":
                    self.max_angular_velocity = Rotation2d.fromDegrees(v.value.value())
                    self.t_pid.setConstraints(
                        TrapezoidProfile.Constraints(
                            self.max_angular_velocity.degrees(),
                            self.max_angular_velocity.degrees() * 5,
                        )
                    )
                elif key[1:].startswith("PID"):
                    const = key.split("/")[0]
                    pid = key[0]
                    if pid == "x":
                        if const == "p":
                            self.x_pid.setP(v.value.value())
                        elif const == "i":
                            self.x_pid.setI(v.value.value())
                        elif const == "d":
                            self.x_pid.setD(v.value.value())
                    elif pid == "y":
                        if const == "p":
                            self.y_pid.setP(v.value.value())
                        elif const == "i":
                            self.y_pid.setI(v.value.value())
                        elif const == "d":
                            self.y_pid.setD(v.value.value())
                    elif pid == "t":
                        if const == "p":
                            self.t_pid.setP(v.value.value())
                        elif const == "i":
                            self.t_pid.setI(v.value.value())
                        elif const == "d":
                            self.t_pid.setD(v.value.value())

                else:
                    reportWarning(f"Got bad key {key}")

        self.nettable.addListener(
            "max_velocity_fps", EventFlags.kValueAll, nettable_listener
        )

        self.nettable.addListener(
            "max_angular_velocity_degps", EventFlags.kValueAll, nettable_listener
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
        fl_position = Translation2d(inchesToMeters(12), inchesToMeters(12))
        fr_position = Translation2d(inchesToMeters(12), -inchesToMeters(12))
        bl_position = Translation2d(-inchesToMeters(12), inchesToMeters(12))
        br_position = Translation2d(-inchesToMeters(12), -inchesToMeters(12))

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
            Pose2d.fromFeet(10, 10, Rotation2d.fromDegrees(0)),
        )

        # self.vision = Vision()

    def periodic(self) -> None:
        position = self.odometry.update(
            self.gyro.get_angle(),
            (
                self.fl.get_position(),
                self.fr.get_position(),
                self.bl.get_position(),
                self.br.get_position(),
            ),
        )

        # self.vision.update_position(self.odometry)

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
        self.nettable.putNumber("state/y (ft)", position.x_feet)
        self.nettable.putNumber("state/theta (deg)", self.get_angle().degrees())

        self.nettable.putNumber("tPID/error (deg)", self.t_pid.getPositionError())
        if c := self.getCurrentCommand():
            self.nettable.putString("Running Command", c.getName())
        else:
            self.nettable.putString("Running Command", "None")

    """getters"""

    def get_kinematics(self) -> SwerveDrive4Kinematics:
        return self.kinematics

    def get_odometry(self) -> SwerveDrive4PoseEstimator:
        return self.odometry

    def get_pose(self) -> Pose2d:
        # mayble flip if on red. Ideally just based on starting position, though
        return self.odometry.getEstimatedPosition()

    def get_angle(self) -> Rotation2d:
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
        return InstantCommand(lambda: self.gyro.reset(new_angle), self)

    def reset_pose(self, pose: Pose2d) -> SequentialCommandGroup:
        return InstantCommand(
            lambda: self.odometry.resetPosition(
                self.get_angle(), self.get_module_positions(), pose
            ),
            self,
        ).andThen(self.reset_gyro(pose.rotation()))

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
        states = self.kinematics.toSwerveModuleStates(speeds, center_of_rotation)
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

    def drive_position(self, position: Pose2d) -> WrapperCommand:
        return self.drive_joystick(
            lambda: self.x_pid.calculate(self.get_pose().X(), position.X()),
            lambda: self.y_pid.calculate(self.get_pose().Y(), position.Y()),
            lambda: self.t_pid.calculate(
                self.get_angle().radians(), position.rotation().radians()
            ),
            lambda: True,
        ).withName("Drive Position")

    def drive_forward(self, dist: feet) -> WrapperCommand:
        position = self.get_pose()
        self.nettable.putNumber("Commanded/xPos (ft)", position.x_feet + dist)
        self.nettable.putNumber("Commanded/yPos (ft)", position.y_feet)
        return self.drive_position(
            Pose2d(feetToMeters(position.x_feet + dist), position.y, self.get_angle()),
        ).withName(f"Drive Forward {dist} ft")

    def defense_mode(self) -> StartEndCommand:
        start_speed = self.max_velocity_mps
        return StartEndCommand(
            lambda: setattr(self, "max_velocity_mps", 500),
            lambda: setattr(self, "max_velocity_mps", start_speed),
        )
