from math import e, pi
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

from wpilib import DriverStation, Timer


class Vision:
    def __init__(self):
        self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)
        self.curr_alliance = DriverStation.Alliance.kBlue
        self.std_devs = (0.01, 0.01, pi / 8)

        self.to_fl = Transform3d(
            Translation3d(inchesToMeters(14), inchesToMeters(14), inchesToMeters(7)),
            Rotation3d.fromDegrees(0, 30, 45),
        )
        self.fl = photonCamera.PhotonCamera("Arducam_FL (1)")

        self.to_fr = Transform3d(
            Translation3d(inchesToMeters(14), -inchesToMeters(14), inchesToMeters(7)),
            Rotation3d.fromDegrees(0, 30, -45),
        )
        self.fr = photonCamera.PhotonCamera("Arducam_FR")

        self.to_bl = Transform3d(
            Translation3d(-inchesToMeters(14), inchesToMeters(14), inchesToMeters(7)),
            Rotation3d.fromDegrees(0, 30, 135),
        )
        self.bl = photonCamera.PhotonCamera("Arducam_BL")

        self.to_br = Transform3d(
            Translation3d(-inchesToMeters(14), -inchesToMeters(14), inchesToMeters(7)),
            Rotation3d.fromDegrees(0, 30, -135),
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
        # if (
        #     DriverStation.getAlliance() == DriverStation.Alliance.kRed
        #     and self.curr_alliance != DriverStation.Alliance.kRed
        # ):
        #     self.curr_alliance = DriverStation.Alliance.kRed
        #     self.field_layout.setOrigin(
        #         Pose3d(
        #             Pose2d(
        #                 self.field_layout.getFieldLength(),
        #                 -self.field_layout.getFieldWidth(),
        #                 Rotation2d.fromDegrees(180),
        #             )
        #         )
        #     )

        #     self.fl_est.fieldTags.setOrigin(self.field_layout.getOrigin())
        #     self.fr_est.fieldTags.setOrigin(self.field_layout.getOrigin())
        #     self.bl_est.fieldTags.setOrigin(self.field_layout.getOrigin())
        #     self.br_est.fieldTags.setOrigin(self.field_layout.getOrigin())
        # elif (
        #     DriverStation.getAlliance() == DriverStation.Alliance.kBlue
        #     and self.curr_alliance != DriverStation.Alliance.kBlue
        # ):
        #     self.curr_alliance = DriverStation.Alliance.kBlue
        #     self.field_layout.setOrigin(Pose3d(Pose2d(0, 0, 0)))

        # self.fl_est.fieldTags.setOrigin(self.field_layout.getOrigin())
        # self.fr_est.fieldTags.setOrigin(self.field_layout.getOrigin())
        # self.bl_est.fieldTags.setOrigin(self.field_layout.getOrigin())
        # self.br_est.fieldTags.setOrigin(self.field_layout.getOrigin())

        # fl = self.fl_est.update()
        # if fl:
        #     odometry.addVisionMeasurement(
        #         fl.estimatedPose.toPose2d(),
        #         fl.timestampSeconds,
        #         self.std_devs,
        #     )

        fr = self.fr_est.update()
        if fr:
            odometry.addVisionMeasurement(
                fr.estimatedPose.toPose2d(),
                fr.timestampSeconds,
                self.std_devs,
            )

        bl = self.bl_est.update()
        if bl:
            odometry.addVisionMeasurement(
                bl.estimatedPose.toPose2d(),
                bl.timestampSeconds,
                self.std_devs,
            )

        br = self.br_est.update()
        if br:
            print(sum(tag.getArea() for tag in br.targetsUsed) / len(br.targetsUsed))
            std_devs = self._calc_std_dev(
                sum(tag.getArea() for tag in br.targetsUsed) / len(br.targetsUsed)
            )
            odometry.addVisionMeasurement(
                br.estimatedPose.toPose2d(),
                br.timestampSeconds,
                std_devs,
            )

        return odometry

    @staticmethod
    def _calc_std_dev(avg_area: float) -> tuple[float, float, float]:
        return (
            x := 0.07 * (e ** (-0.5 * avg_area)),
            x,
            (pi) ** avg_area,
        )
