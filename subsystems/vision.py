from commands2 import Subsystem
from photonlibpy import photonCamera, photonPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpimath.units import inchesToMeters
from wpimath.geometry import (
    Translation3d,
    Transform3d,
    Rotation3d,
    Pose3d,
    Pose2d,
    Rotation2d,
)
from wpimath.estimator import SwerveDrive4PoseEstimator

from wpilib import DriverStation


class Vision:
    def __init__(self):
        self.field_layout = AprilTagFieldLayout()
        self.curr_alliance = DriverStation.Alliance.kBlue

        self.to_fl = Transform3d(
            Translation3d(inchesToMeters(14), inchesToMeters(7), -inchesToMeters(14)),
            Rotation3d.fromDegrees(0, 45, 45),
        )
        self.fl = photonCamera.PhotonCamera("Arducam_FL")

        self.to_fr = Transform3d(
            Translation3d(inchesToMeters(14), inchesToMeters(7), inchesToMeters(14)),
            Rotation3d.fromDegrees(0, 35, -45),
        )
        self.fr = photonCamera.PhotonCamera("Arducam_FR")

        self.to_bl = Transform3d(
            Translation3d(-inchesToMeters(14), inchesToMeters(7), -inchesToMeters(14)),
            Rotation3d.fromDegrees(0, 45, 135),
        )
        self.bl = photonCamera.PhotonCamera("Arducam_BL")

        self.to_br = Transform3d(
            Translation3d(-inchesToMeters(14), inchesToMeters(7), inchesToMeters(14)),
            Rotation3d.fromDegrees(0, 45, -135),
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

    def update_position(
        self, odometry: SwerveDrive4PoseEstimator
    ) -> SwerveDrive4PoseEstimator:
        if (
            DriverStation.getAlliance() == DriverStation.Alliance.kRed
            and self.curr_alliance != DriverStation.Alliance.kRed
        ):
            self.curr_alliance = DriverStation.Alliance.kRed
            self.field_layout.setOrigin(
                Pose3d(
                    Pose2d(
                        self.field_layout.getFieldLength(),
                        -self.field_layout.getFieldWidth(),
                        Rotation2d.fromDegrees(180),
                    )
                )
            )

            self.fl_est.fieldTags.setOrigin(self.field_layout.getOrigin())
            self.fr_est.fieldTags.setOrigin(self.field_layout.getOrigin())
            self.bl_est.fieldTags.setOrigin(self.field_layout.getOrigin())
            self.br_est.fieldTags.setOrigin(self.field_layout.getOrigin())
        elif (
            DriverStation.getAlliance() == DriverStation.Alliance.kBlue
            and self.curr_alliance != DriverStation.Alliance.kBlue
        ):
            self.curr_alliance = DriverStation.Alliance.kBlue
            self.field_layout.setOrigin(Pose3d(Pose2d(0, 0, 0)))

            self.fl_est.fieldTags.setOrigin(self.field_layout.getOrigin())
            self.fr_est.fieldTags.setOrigin(self.field_layout.getOrigin())
            self.bl_est.fieldTags.setOrigin(self.field_layout.getOrigin())
            self.br_est.fieldTags.setOrigin(self.field_layout.getOrigin())
        fl_res = self.fl.getLatestResult()
        fl = self.fl_est.update(fl_res)
        if fl:
            odometry.addVisionMeasurement(
                fl.estimatedPose.toPose2d(),
                fl.timestampSeconds,
                (0.01, 0.01, float("infinity")),
            )
            print("asdflasjdfak;ldsjkladfsjk;ladsjfkl;adfsj")

        fr_res = self.fr.getLatestResult()
        fr = self.fr_est.update(fr_res)
        if fr:
            odometry.addVisionMeasurement(
                fr.estimatedPose.toPose2d(),
                fr.timestampSeconds,
                (0.01, 0.01, float("infinity")),
            )
            print("asdflasjdfak;ldsjkladfsjk;ladsjfkl;adfsj")

        bl_res = self.bl.getLatestResult()
        bl = self.bl_est.update(bl_res)
        if bl is not None:
            odometry.addVisionMeasurement(
                bl.estimatedPose.toPose2d(),
                bl.timestampSeconds,
                (0.01, 0.01, float("infinity")),
            )
            print("asdflasjdfak;ldsjkladfsjk;ladsjfkl;adfsj")

        br_res = self.br.getLatestResult()
        br = self.br_est.update(br_res)
        if br:
            odometry.addVisionMeasurement(
                br.estimatedPose.toPose2d(),
                br.timestampSeconds,
                (0.01, 0.01, float("infinity")),
            )
            print("asdflasjdfak;ldsjkladfsjk;ladsjfkl;adfsj")

        return odometry
