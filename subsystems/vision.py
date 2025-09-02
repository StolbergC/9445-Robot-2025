from typing import Callable

from commands2 import Command, InstantCommand, Subsystem
from wpilib import RobotBase, SmartDashboard
from math import e, pi
from ntcore import NetworkTableInstance
from photonlibpy import photonCamera, photonPoseEstimator


from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.units import inchesToMeters
from wpimath.geometry import (
    Pose2d,
    Translation3d,
    Transform3d,
    Rotation3d,
    Pose3d,
    Rotation2d,
)
from wpimath.units import (
    seconds,
    meters,
    radians,
    meters_per_second,
    degrees_per_second,
)
from wpimath.kinematics import ChassisSpeeds

from wpilib import RobotBase


class Vision(Subsystem):
    enabled: bool = True

    strategy: photonPoseEstimator.PoseStrategy = (
        photonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    )

    max_omega: degrees_per_second = 90
    max_velocity: meters_per_second = 4

    std_devs = (1, 1, pi / 2)
    std_dev_target_factor = 1.75

    def __init__(
        self,
        log_vision_measurement: Callable[
            [Pose2d, seconds, tuple[meters, meters, radians]], None
        ],
        get_robot_pose: Callable[[], Pose2d],
        get_robot_velocity: Callable[[], ChassisSpeeds],
    ):
        self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)
        self.log_vision_measurement = log_vision_measurement
        self.get_robot_pose = get_robot_pose
        self.get_speeds = get_robot_velocity

        self.nettable = NetworkTableInstance.getDefault().getTable("Vision")
        self.sightline_pub = self.nettable.getStructArrayTopic(
            "VisibleTargets", Pose3d
        ).publish()
        self.pose_est_pub = self.nettable.getStructArrayTopic(
            "EstimatedPoses", Pose3d
        ).publish()

        self.to_fl = Transform3d(
            Translation3d(inchesToMeters(14), inchesToMeters(14), inchesToMeters(7)),
            Rotation3d.fromDegrees(0, 15, -45),
        )
        self.fl = photonCamera.PhotonCamera("Arducam_FL (1)")

        self.to_fr = Transform3d(
            Translation3d(inchesToMeters(14), -inchesToMeters(14), inchesToMeters(7)),
            Rotation3d.fromDegrees(0, 15, 45),
        )
        self.fr = photonCamera.PhotonCamera("Arducam_FR")

        self.to_bl = Transform3d(
            Translation3d(-inchesToMeters(14), inchesToMeters(14), inchesToMeters(7)),
            Rotation3d.fromDegrees(0, 15, -135),
        )
        self.bl = photonCamera.PhotonCamera("Arducam_BL")

        self.to_br = Transform3d(
            Translation3d(-inchesToMeters(14), -inchesToMeters(14), inchesToMeters(7)),
            Rotation3d.fromDegrees(0, 15, 135),
        )
        self.br = photonCamera.PhotonCamera("Arducam_BR")

        self.fl_est = photonPoseEstimator.PhotonPoseEstimator(
            self.field_layout,
            self.strategy,
            self.fl,
            self.to_fl,
        )
        self.fr_est = photonPoseEstimator.PhotonPoseEstimator(
            self.field_layout,
            self.strategy,
            self.fr,
            self.to_fr,
        )
        self.bl_est = photonPoseEstimator.PhotonPoseEstimator(
            self.field_layout,
            self.strategy,
            self.bl,
            self.to_bl,
        )
        self.br_est = photonPoseEstimator.PhotonPoseEstimator(
            self.field_layout,
            self.strategy,
            self.br,
            self.to_br,
        )

        if RobotBase.isSimulation():
            from photonlibpy.simulation import (
                visionSystemSim,
                simCameraProperties,
                photonCameraSim,
            )

            self.vision_sim = visionSystemSim.VisionSystemSim("Vision Sim")
            self.vision_sim.addAprilTags(self.field_layout)
            # TODO: Determine based on our cameras
            camera_properties = simCameraProperties.SimCameraProperties()
            camera_properties.setCalibrationFromFOV(
                1920,
                1080,
                Rotation2d.fromDegrees(105),
                # 1280,
                # 720,
                # Rotation2d.fromDegrees(68.5),
            )
            # camera_properties.setCalibError(0.35, 0.1)
            camera_properties.setFPS(15)
            camera_properties.setAvgLatency(50 / 1000)
            camera_properties.setLatencyStdDev(15 / 1000)

            fl_sim = photonCameraSim.PhotonCameraSim(self.fl, camera_properties)
            fr_sim = photonCameraSim.PhotonCameraSim(self.fr, camera_properties)
            bl_sim = photonCameraSim.PhotonCameraSim(self.bl, camera_properties)
            br_sim = photonCameraSim.PhotonCameraSim(self.br, camera_properties)

            self.vision_sim.addCamera(fl_sim, self.to_fl)
            self.vision_sim.addCamera(fr_sim, self.to_fr)
            self.vision_sim.addCamera(bl_sim, self.to_bl)
            self.vision_sim.addCamera(br_sim, self.to_br)

            for k, v in [
                ("fl", self.to_fl),
                ("fr", self.to_fr),
                ("bl", self.to_bl),
                ("br", self.to_br),
            ]:
                self.nettable.putNumber(f"{k}/x", v.X())
                self.nettable.putNumber(f"{k}/y", v.Y())
                self.nettable.putNumber(f"{k}/z", v.Z())
                self.nettable.putNumber(f"{k}/roll", v.rotation().x)
                self.nettable.putNumber(f"{k}/pitch", v.rotation().y)
                self.nettable.putNumber(f"{k}/yaw", v.rotation().z)

            # Not yet implemented in photonlibpy
            # See https://github.com/BrysonSmith15/PhotonvisionWireframeNetworkTables
            # for instructions on viewing simulated vision data with wireframe
            # fl_sim.enableDrawWireframe(True)
            # fr_sim.enableDrawWireframe(True)
            # bl_sim.enableDrawWireframe(True)
            # br_sim.enableDrawWireframe(True)
            SmartDashboard.putData(self.vision_sim.getDebugField())

    def periodic(self) -> None:
        self.nettable.putBoolean("Enabled", self.enabled)
        speeds = self.get_speeds()
        if (
            not self.enabled
            or abs(speeds.omega_dps) > self.max_omega
            or abs(speeds.vx) >= self.max_velocity
            or abs(speeds.vy) >= self.max_velocity
        ):
            self.sightline_pub.set([])
            self.pose_est_pub.set([])
            return
        seen_ids: list[int] = []
        estimated_poses: list[Pose3d] = []

        fr_est = self.fr_est.update()
        if fr_est:
            if len(fr_est.targetsUsed) > 0:
                fr_pose = fr_est.estimatedPose.toPose2d()
                dist = fr_est.estimatedPose.translation().distance(
                    self.to_fr.translation()
                )
                self.log_vision_measurement(
                    fr_pose,
                    fr_est.timestampSeconds,
                    self._calc_std_dev(dist, len(fr_est.targetsUsed)),
                )

                seen_ids.extend([target.fiducialId for target in fr_est.targetsUsed])

                estimated_poses.append(fr_est.estimatedPose)

        fl_est = self.fl_est.update()
        if fl_est:
            if len(fl_est.targetsUsed) > 0:
                fl_pose = fl_est.estimatedPose.toPose2d()
                dist = fl_est.estimatedPose.translation().distance(
                    self.to_fl.translation()
                )
                self.log_vision_measurement(
                    fl_pose,
                    fl_est.timestampSeconds,
                    self._calc_std_dev(dist, len(fl_est.targetsUsed)),
                )

                seen_ids.extend([target.fiducialId for target in fl_est.targetsUsed])
                estimated_poses.append(fl_est.estimatedPose)

        bl_est = self.bl_est.update()
        if bl_est:
            if len(bl_est.targetsUsed) > 0:
                bl_pose = bl_est.estimatedPose.toPose2d()
                dist = bl_est.estimatedPose.translation().distance(
                    self.to_bl.translation()
                )
                self.log_vision_measurement(
                    bl_pose,
                    bl_est.timestampSeconds,
                    self._calc_std_dev(dist, len(bl_est.targetsUsed)),
                )

                seen_ids.extend([target.fiducialId for target in bl_est.targetsUsed])
                estimated_poses.append(bl_est.estimatedPose)

        br_est = self.br_est.update()
        if br_est:
            if len(br_est.targetsUsed) > 0:
                br_pose = br_est.estimatedPose.toPose2d()
                dist = br_est.estimatedPose.translation().distance(
                    self.to_br.translation()
                )
                self.log_vision_measurement(
                    br_pose,
                    br_est.timestampSeconds,
                    self._calc_std_dev(dist, len(br_est.targetsUsed)),
                )

                seen_ids.extend([target.fiducialId for target in br_est.targetsUsed])
                estimated_poses.append(br_est.estimatedPose)

        self.sightline_pub.set([self.field_layout.getTagPose(id) for id in seen_ids])
        self.pose_est_pub.set(estimated_poses)

    def simulationPeriodic(self) -> None:
        self.vision_sim.update(self.get_robot_pose())

    # calculate standard deviation based on the target distance
    def _calc_std_dev(
        self, dist: float, targets_used: int = 1
    ) -> tuple[float, float, float]:
        # stddevs increase with distance, 0 is full trust. Distance is in meters
        return (
            (self.std_dev_target_factor**-targets_used) * self.std_devs[0] * dist**2,
            (self.std_dev_target_factor**-targets_used) * self.std_devs[1] * dist**2,
            (self.std_dev_target_factor**-targets_used) * self.std_devs[2] * dist**2,
        )

    def disable_measurements(self) -> None:
        self.enabled = False

    def enable_measurements(self) -> None:
        self.enabled = True

    def toggle_vision_measurements(self) -> None:
        self.enabled = not self.enabled

    def toggle_vision_measurements_command(self) -> Command:
        return InstantCommand(self.toggle_vision_measurements)
