package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TunableConstant;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {

  public static class Config {
    public Transform3d rightCameraToRobot;
    public Transform3d leftCameraToRobot;
  }

  // Method to be called when a new pose estimate is ready
  private Optional<Consumer<StampedPose2d>> updatePoseCallback = Optional.empty();

  public record StampedPose2d(Pose2d pose, double timestamp) {}

  VisionProcessing leftVision;
  VisionProcessing rightVision;

  private long lastLogTime;
  private Alliance alliance;
  public Pose2d coralStationLeft;
  public Pose2d coralStationRight;
  Pose2d reefCenter;
  private static Transform2d coralStationTransform =
      new Transform2d(Inches.of(20), Inches.of(11.628), Rotation2d.k180deg);

  private double latest_reef_tag_pitch_right_cam = 1058.0;
  private double latest_reef_tag_yaw_right_cam = 1058.0;
  private double latest_reef_tag_pitch_left_cam = 1058.0;
  private double latest_reef_tag_yaw_left_cam = 1058.0;

  AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public void updateAlliance(Alliance alliance) {
    this.alliance = alliance;

    if (alliance == Alliance.Red) {
      coralStationLeft =
          fieldLayout.getTagPose(1).get().toPose2d().transformBy(coralStationTransform);
      coralStationRight =
          fieldLayout.getTagPose(2).get().toPose2d().transformBy(coralStationTransform);
      reefCenter = new Pose2d(Inches.of(513.625), Inches.of(158.5), Rotation2d.fromDegrees(-150));
    } else {
      coralStationLeft =
          fieldLayout.getTagPose(13).get().toPose2d().transformBy(coralStationTransform);
      coralStationRight =
          fieldLayout.getTagPose(12).get().toPose2d().transformBy(coralStationTransform);
      reefCenter = new Pose2d(Inches.of(176.75), Inches.of(158.5), Rotation2d.fromDegrees(30));
    }
  }

  public class VisionProcessing {
    PhotonPoseEstimator primaryPoseEstimator;
    PhotonPoseEstimator secondaryPoseEstimator;
    PhotonCamera camera;
    double poseAmbiguity;

    TunableConstant x;
    TunableConstant y;
    TunableConstant z;

    TunableConstant yaw;
    TunableConstant pitch;
    TunableConstant roll;

    ArrayList<TunableConstant> tunableList;

    Optional<StampedPose2d> robotPose = Optional.empty();

    // Initialize the PhotonPoseEstimator
    VisionProcessing(String cameraName, Transform3d cameraToRobot) {
      camera = new PhotonCamera(cameraName);
      try {
        primaryPoseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraToRobot);

        secondaryPoseEstimator = new PhotonPoseEstimator(
            fieldLayout, PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE, cameraToRobot);

      } catch (Exception e) {
        e.printStackTrace();
      }

      this.x = new TunableConstant(cameraName, cameraToRobot.getX());
      this.y = new TunableConstant(cameraName, cameraToRobot.getY());
      this.z = new TunableConstant(cameraName, cameraToRobot.getZ());
      this.yaw = new TunableConstant(
          cameraName, cameraToRobot.getRotation().getMeasureZ().in(Degree));
      this.pitch = new TunableConstant(
          cameraName, cameraToRobot.getRotation().getMeasureY().in(Degree));
      this.roll = new TunableConstant(
          cameraName, cameraToRobot.getRotation().getMeasureX().in(Degree));
      this.tunableList = new ArrayList<>(List.of(x, y, z, yaw, pitch, roll));
    }

    public void rebuildPoseEstimators() {
      Transform3d cameraToRobot = new Transform3d(
          x.getAndUpdate(),
          y.getAndUpdate(),
          z.getAndUpdate(),
          new Rotation3d(roll.getAndUpdate(), pitch.getAndUpdate(), yaw.getAndUpdate()));

      primaryPoseEstimator.setRobotToCameraTransform(cameraToRobot);
      secondaryPoseEstimator.setRobotToCameraTransform(cameraToRobot);
    }

    private static boolean isValidPose(Pose3d pose) {
      double z = pose.getZ();
      Rotation3d rotation = pose.getRotation();

      double roll = rotation.getX();
      double pitch = rotation.getY();
      // double yaw = rotation.getZ();

      if (Math.abs(roll) > Math.PI / 2 || Math.abs(pitch) > Math.PI / 2) {
        return false;
      }

      if (z < -0.1) {
        return false;
      }

      double maxTilt = Math.toRadians(30);
      if (Math.abs(roll) > maxTilt || Math.abs(pitch) > maxTilt) {
        return false;
      }

      return true;
    }

    private Optional<StampedPose2d> readRobotPoseFromNT() {

      List<PhotonPipelineResult> resultList = camera.getAllUnreadResults();

      if (resultList.isEmpty()) {
        if (Objects.equals(this.camera.getName(), "left")) {
          latest_reef_tag_pitch_right_cam = 1058.0;
          latest_reef_tag_yaw_right_cam = 1058.0;
        } else if (Objects.equals(this.camera.getName(), "right")) {
          latest_reef_tag_pitch_left_cam = 1058.0;
          latest_reef_tag_yaw_left_cam = 1058.0;
        }
        return Optional.empty();
      }

      PhotonPipelineResult result = resultList.get(resultList.size() - 1);

      if (result.hasTargets()) {
        // Update the pose estimator with the latest camera data
        EstimatedRobotPose estimatedPose = primaryPoseEstimator.update(result).orElse(null);
        poseAmbiguity = 0;
        if (estimatedPose == null) {
          estimatedPose = secondaryPoseEstimator.update(result).orElse(null);
          poseAmbiguity = result.getBestTarget().poseAmbiguity;
          if (estimatedPose == null || poseAmbiguity > 0.18) {
            return Optional.empty();
          }
        } else {
          poseAmbiguity = result.getBestTarget().poseAmbiguity;
          if (poseAmbiguity > 0.28) {
            return Optional.empty();
          }
        }

        if (this.robotPose.isPresent()) {
          if (Math.abs(this.robotPose.get().pose.getX() - estimatedPose.estimatedPose.getX()) > 0.1
              || Math.abs(this.robotPose.get().pose.getY() - estimatedPose.estimatedPose.getY())
                  > 0.1) {
            return Optional.empty();
          }
        }

        Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
        Pose3d pose3d = estimatedPose.estimatedPose;
        Logger.recordOutput(
            "Pose3dZTranslation", pose3d.getTranslation().getMeasureZ().baseUnitMagnitude());
        Logger.recordOutput(
            "Pose3dXTranslation", pose3d.getTranslation().getMeasureX().baseUnitMagnitude());
        Logger.recordOutput(
            "Pose3dYTranslation", pose3d.getTranslation().getMeasureY().baseUnitMagnitude());
        Logger.recordOutput(camera.getName() + "/2DVisionPoseUnfiltered", pose2d);
        Logger.recordOutput(camera.getName() + "/3DVisionPoseUnfiltered", pose3d);

        if (isValidPose(pose3d)) {
          Logger.recordOutput(camera.getName() + "/2DVisionPoseFiltered", pose2d);
          Logger.recordOutput(camera.getName() + "/3DVisionPoseFiltered", pose3d);
          return Optional.of(
              new StampedPose2d(pose2d, Utils.fpgaToCurrentTime(result.getTimestampSeconds())));
        }
      }
      return Optional.empty();
    }

    public void updateRobotPose() {
      this.robotPose = readRobotPoseFromNT();
    }

    public Optional<StampedPose2d> getPose() {
      return robotPose;
    }

    public void rebuildCameras() {}
  }

  public Vision(Config config) {
    rightVision = new VisionProcessing("right", config.rightCameraToRobot);
    leftVision = new VisionProcessing("left", config.leftCameraToRobot);
  }

  public void onPoseUpdate(Consumer<StampedPose2d> cb) {
    updatePoseCallback = Optional.of(cb);
  }

  public double getLatestReefTagPitchRightCam() {
    return latest_reef_tag_pitch_right_cam;
  }

  public double getLatestReefTagYawRightCam() {
    return latest_reef_tag_yaw_right_cam;
  }

  public double getLatestReefTagPitchLeftCam() {
    return latest_reef_tag_pitch_left_cam;
  }

  public double getLatestReefTagYawLeftCam() {
    return latest_reef_tag_yaw_left_cam;
  }

  // Isn't this more like "getAveragePoseFromBothCameras"? - Ben
  public StampedPose2d chooseBestPoseBetweenCameras(
      StampedPose2d backLeftCameraPose, StampedPose2d backRightCameraPose) {
    Translation2d averageTranslation = backLeftCameraPose
        .pose
        .getTranslation()
        .plus(backRightCameraPose.pose.getTranslation())
        .div(2.0);
    Rotation2d averageRotation = backLeftCameraPose
        .pose
        .getRotation()
        .interpolate(backRightCameraPose.pose.getRotation(), 0.5);

    return new StampedPose2d(
        new Pose2d(averageTranslation, averageRotation),
        (backLeftCameraPose.timestamp + backRightCameraPose.timestamp) / 2);
  }

  // public StampedPose2d chooseBestPoseBetween2CamerasToClosestTarget(
  //     StampedPose2d frontCameraPose, StampedPose2d backCameraPose) {
  //   Translation2d averageTranslation = frontCameraPose
  //       .pose
  //       .getTranslation()
  //       .plus(backCameraPose.pose.getTranslation())
  //       .div(2.0);
  //   Rotation2d averageRotation =
  //       frontCameraPose.pose.getRotation().interpolate(backCameraPose.pose.getRotation(), 0.5);

  //   var distanceToReef = averageTranslation.getDistance(this.reefCenter.getTranslation());
  //   var distanceToCoralStationLeft =
  //       averageTranslation.getDistance(this.coralStationLeft.getTranslation());
  //   var distanceToCoralStationRight =
  //       averageTranslation.getDistance(this.coralStationRight.getTranslation());

  //   if (distanceToReef < distanceToCoralStationLeft
  //       && distanceToReef < distanceToCoralStationRight) {
  //     return backCameraPose;
  //   } else if (distanceToCoralStationLeft < distanceToReef
  //       || distanceToCoralStationRight < distanceToReef) {
  //     return frontCameraPose;
  //   }
  //   return new StampedPose2d(
  //       new Pose2d(averageTranslation, averageRotation),
  //       (backCameraPose.timestamp + frontCameraPose.timestamp) / 2);
  // }

  // public StampedPose2d chooseBestPoseBetween3CamerasToClosestTarget(
  //     StampedPose2d frontCameraPose,
  //     StampedPose2d backLeftCameraPose,
  //     StampedPose2d backRightCameraPose) {
  //   StampedPose2d bestFirstPose =
  //       chooseBestPoseBetween2CamerasToClosestTarget(frontCameraPose, backLeftCameraPose);
  //   StampedPose2d bestSecondPose =
  //       chooseBestPoseBetween2CamerasToClosestTarget(frontCameraPose, backRightCameraPose);
  //   return chooseBestPoseBetween2CamerasToClosestTarget(bestFirstPose, bestSecondPose);
  // }

  @Override
  public void periodic() {
    leftVision.updateRobotPose();
    rightVision.updateRobotPose();

    lastLogTime = System.currentTimeMillis();
    var leftRobotPose = leftVision.getPose();
    var rightRobotPose = rightVision.getPose();

    // if (updatePoseCallback.isPresent()) {
    //   if (backLeftRobotPose.isPresent() && backRightRobotPose.isPresent()) {
    //     updatePoseCallback
    //         .get()
    //         .accept(chooseBestPoseBetweenBothBackCameras(
    //             backLeftRobotPose.get(), backRightRobotPose.get()));
    //   } else if (backLeftRobotPose.isPresent()) {
    //     updatePoseCallback.get().accept((backLeftRobotPose.get()));
    //   } else if (backRightRobotPose.isPresent()) {
    //     updatePoseCallback.get().accept((backRightRobotPose.get()));
    //   }
    // }

    if (updatePoseCallback.isPresent()) {
      if (leftRobotPose.isPresent() && rightRobotPose.isPresent()) {
        updatePoseCallback
            .get()
            .accept(chooseBestPoseBetweenCameras(leftRobotPose.get(), rightRobotPose.get()));
      } else if (leftRobotPose.isPresent()) {
        updatePoseCallback.get().accept((leftRobotPose.get()));
      } else if (rightRobotPose.isPresent()) {
        updatePoseCallback.get().accept((rightRobotPose.get()));
      }
    }
  }

  public void testPeriodic() {
    TunableConstant.ifChanged((dub) -> leftVision.rebuildPoseEstimators(), leftVision.tunableList);
    TunableConstant.ifChanged(
        (dub) -> rightVision.rebuildPoseEstimators(), rightVision.tunableList);
  }
}
