from wpilib import RobotBase
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
from wpimath.estimator import SwerveDrive4PoseEstimator

from wpilib import Field2d, RobotBase


class Vision:
    def __init__(self):
        self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)
        self.std_devs = (0.01, 0.01, pi / 8)

        self.nettable = NetworkTableInstance.getDefault().getTable("Vision")
        self.sightline_pub = self.nettable.getStructArrayTopic(
            "VisibleTargets", Pose3d
        ).publish()

        self.to_fl = Transform3d(
            Translation3d(inchesToMeters(14), inchesToMeters(14), inchesToMeters(7)),
            Rotation3d.fromDegrees(0, -15, 45),
        )
        self.fl = photonCamera.PhotonCamera("Arducam_FL (1)")

        self.to_fr = Transform3d(
            Translation3d(inchesToMeters(14), -inchesToMeters(14), inchesToMeters(7)),
            Rotation3d.fromDegrees(0, -15, -45),
        )
        self.fr = photonCamera.PhotonCamera("Arducam_FR")

        self.to_bl = Transform3d(
            Translation3d(-inchesToMeters(14), inchesToMeters(14), inchesToMeters(7)),
            Rotation3d.fromDegrees(0, -15, 135),
        )
        self.bl = photonCamera.PhotonCamera("Arducam_BL")

        self.to_br = Transform3d(
            Translation3d(-inchesToMeters(14), -inchesToMeters(14), inchesToMeters(7)),
            Rotation3d.fromDegrees(0, -15, -135),
        )
        self.br = photonCamera.PhotonCamera("Arducam_BR")

        self.fl_est = photonPoseEstimator.PhotonPoseEstimator(
            self.field_layout,
            photonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.fl,
            self.to_fl,
        )
        self.fr_est = photonPoseEstimator.PhotonPoseEstimator(
            self.field_layout,
            photonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.fr,
            self.to_fr,
        )
        self.bl_est = photonPoseEstimator.PhotonPoseEstimator(
            self.field_layout,
            photonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.bl,
            self.to_bl,
        )
        self.br_est = photonPoseEstimator.PhotonPoseEstimator(
            self.field_layout,
            photonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
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

    def update_position(self, odometry: SwerveDrive4PoseEstimator) -> None:
        seen_ids: list[int] = []

        fr_est = self.fr_est.update()
        if fr_est:
            if len(fr_est.targetsUsed) > 0:
                fr_pose = fr_est.estimatedPose.toPose2d()
                dist = fr_est.estimatedPose.translation().distance(
                    self.to_fr.translation()
                )
                odometry.addVisionMeasurement(
                    fr_pose,
                    fr_est.timestampSeconds,
                    self._calc_std_dev(dist, len(fr_est.targetsUsed)),
                )
                seen_ids.extend([target.fiducialId for target in fr_est.targetsUsed])

        fl_est = self.fl_est.update()
        if fl_est:
            if len(fl_est.targetsUsed) > 0:
                fl_pose = fl_est.estimatedPose.toPose2d()
                dist = fl_est.estimatedPose.translation().distance(
                    self.to_fl.translation()
                )
                odometry.addVisionMeasurement(
                    fl_pose,
                    fl_est.timestampSeconds,
                    self._calc_std_dev(dist, len(fl_est.targetsUsed)),
                )
                seen_ids.extend([target.fiducialId for target in fl_est.targetsUsed])

        bl_est = self.bl_est.update()
        if bl_est:
            if len(bl_est.targetsUsed) > 0:
                bl_pose = bl_est.estimatedPose.toPose2d()
                dist = bl_est.estimatedPose.translation().distance(
                    self.to_bl.translation()
                )
                odometry.addVisionMeasurement(
                    bl_pose,
                    bl_est.timestampSeconds,
                    self._calc_std_dev(dist, len(bl_est.targetsUsed)),
                )
                seen_ids.extend([target.fiducialId for target in bl_est.targetsUsed])

        br_est = self.br_est.update()
        if br_est:
            if len(br_est.targetsUsed) > 0:
                br_pose = br_est.estimatedPose.toPose2d()
                dist = br_est.estimatedPose.translation().distance(
                    self.to_br.translation()
                )
                odometry.addVisionMeasurement(
                    br_pose,
                    br_est.timestampSeconds,
                    self._calc_std_dev(dist, len(br_est.targetsUsed)),
                )
                seen_ids.extend([target.fiducialId for target in br_est.targetsUsed])

        self.sightline_pub.set([self.field_layout.getTagPose(id) for id in seen_ids])

    def sim_update(self, pose: Pose2d) -> Field2d | None:
        if RobotBase.isSimulation():
            self.vision_sim.update(pose)
            return self.vision_sim.getDebugField()

    # calculate standard deviation based on the target distance
    def _calc_std_dev(
        self, dist: float, targets_used: int = 1
    ) -> tuple[float, float, float]:
        # stddevs increase with distance, 0 is full trust. Distance is in meters
        return (
            (e**-targets_used) * self.std_devs[0] * dist**2,
            (e**-targets_used) * self.std_devs[1] * dist**2,
            (e**-targets_used) * self.std_devs[2] * dist**2,
        )
