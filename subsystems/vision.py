from commands2 import Subsystem
from photonlibpy import photonCamera, photonPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.units import inchesToMeters
from wpimath.geometry import Translation3d, Transform3d, Rotation3d, Pose3d
from wpimath.estimator import SwerveDrive4PoseEstimator

class Vision:
    def __init__(self):
        self.field_layout = AprilTagFieldLayout()

        self.to_fl = Transform3d(Translation3d(inchesToMeters(14), inchesToMeters(7), -inchesToMeters(14)), Rotation3d.fromDegrees(0, 45, 45))
        self.fl = photonCamera.PhotonCamera("fl camera")

        self.to_fr = Transform3d(Translation3d(inchesToMeters(14), inchesToMeters(7), inchesToMeters(14)), Rotation3d.fromDegrees(0, 35, -45))
        self.fr = photonCamera.PhotonCamera("fr camera")

        self.to_bl = Transform3d(Translation3d(-inchesToMeters(14), inchesToMeters(7), -inchesToMeters(14)), Rotation3d.fromDegrees(0, 45, 135))
        self.bl = photonCamera.PhotonCamera("bl camera")

        self.to_br = Transform3d(Translation3d(-inchesToMeters(14), inchesToMeters(7), inchesToMeters(14)), Rotation3d.fromDegrees(0, 45, -135))
        self.br = photonCamera.PhotonCamera("br camera")

        self.fl_est = photonPoseEstimator.PhotonPoseEstimator(self.field_layout, photonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, self.fl, self.to_fl) 
        self.fr_est = photonPoseEstimator.PhotonPoseEstimator(self.field_layout, photonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, self.fr, self.to_fr) 
        self.bl_est = photonPoseEstimator.PhotonPoseEstimator(self.field_layout, photonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, self.bl, self.to_bl) 
        self.br_est = photonPoseEstimator.PhotonPoseEstimator(self.field_layout, photonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, self.br, self.to_br) 

    def update_position(self, odometry: SwerveDrive4PoseEstimator) -> None:
        fl = self.fl_est.update()
        if fl:
            odometry.addVisionMeasurement(fl.estimatedPose.toPose2d(), fl.timestampSeconds)

        fr = self.fl_est.update()
        if fr:
            odometry.addVisionMeasurement(fr.estimatedPose.toPose2d(), fr.timestampSeconds)

        bl = self.fl_est.update()
        if bl:
            odometry.addVisionMeasurement(bl.estimatedPose.toPose2d(), bl.timestampSeconds)

        br = self.fl_est.update()
        if br:
            odometry.addVisionMeasurement(br.estimatedPose.toPose2d(), br.timestampSeconds)



