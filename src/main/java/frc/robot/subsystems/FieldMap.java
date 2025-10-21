package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FieldMap extends SubsystemBase {
  static AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  Alliance alliance;

  // Left and Right are relative to the driver
  public Pose2d coralStationLeft;
  public Pose2d coralStationRight;

  public Pose2d coralStationLeftIntermediate;
  public Pose2d coralStationMiddleLeftIntermediate;
  public Pose2d coralStationMiddleRightIntermediate;
  public Pose2d coralStationRightIntermediate;

  Pose2d reefCenter;
  double angleToReef = 0;
  double angleToTag = 0;

  // These are old and will need to be updated for Riptide
  private static final Transform2d coralStationTransform =
      new Transform2d(Inches.of(16.5), Inches.of(12.628), Rotation2d.kZero);

  // old left: -9.75
  // old right: 3.25
  private final static double LEFT_REEF_Y = -6;

  private static final Transform2d leftReefTransform =
      new Transform2d(Inches.of(19.5), Inches.of(LEFT_REEF_Y), Rotation2d.k180deg);
  private static final Transform2d farLeftReefTransform =
      new Transform2d(Inches.of(48), Inches.of(LEFT_REEF_Y), Rotation2d.k180deg);

  private static final Transform2d L2L3LeftReefTransform =
      new Transform2d(Inches.of(17), Inches.of(LEFT_REEF_Y), Rotation2d.k180deg);

  private final static double RIGHT_REEF_Y = 6;
  private static final Transform2d rightReefTransform =
      new Transform2d(Inches.of(19.5), Inches.of(RIGHT_REEF_Y), Rotation2d.k180deg);
  private static final Transform2d farRightReefTransform =
      new Transform2d(Inches.of(48), Inches.of(RIGHT_REEF_Y), Rotation2d.k180deg);
  private static final Transform2d L2L3RightReefTransform =
      new Transform2d(Inches.of(17), Inches.of(RIGHT_REEF_Y), Rotation2d.k180deg);

  private static final Transform2d farAlgaeTransform =
      new Transform2d(Inches.of(36), Inches.of(0), Rotation2d.k180deg);
  private static final Transform2d closeAlgaeTransform =
      new Transform2d(Inches.of(16), Inches.of(0), Rotation2d.k180deg);

  private static final double leftPolePitch = 2.5;
  private static final double leftPoleYaw = 31.5;
  private static final double rightPolePitch = -0.9;
  private static final double rightPoleYaw = -14.2;

  public FieldMap(Alliance alliance) {
    this.alliance = alliance;
    if (alliance == Alliance.Red) {
      coralStationLeft =
          fieldLayout.getTagPose(1).get().toPose2d().transformBy(coralStationTransform);
      coralStationRight =
          fieldLayout.getTagPose(2).get().toPose2d().transformBy(coralStationTransform);
      reefCenter = new Pose2d(Inches.of(513.625), Inches.of(158.5), Rotation2d.fromDegrees(-150));
      coralStationLeftIntermediate = new Pose2d(Meters.of(13.8), Meters.of(1.5), Rotation2d.kZero);
      coralStationRightIntermediate = new Pose2d(Meters.of(13.8), Meters.of(6.5), Rotation2d.kZero);

      // leftrelativeX = new Pose2d(Meters.of(12.44), Meters.of(5.21), new
      // Rotation2d(Degrees.of(-60))).relativeTo(ReefFace.Two.redPose).getX();
      // leftRelativeY = new Pose2d(Meters.of(12.44), Meters.of(5.21), new
      // Rotation2d(Degrees.of(-60))).relativeTo(ReefFace.Two.redPose).getY();

      // rightrelativeX = new Pose2d(Meters.of(12.31), Meters.of(5.10), new
      // Rotation2d(Degrees.of(-60))).relativeTo(ReefFace.Two.redPose).getX();
      // rightRelativeY = new Pose2d(Meters.of(12.31), Meters.of(5.10), new
      // Rotation2d(Degrees.of(-60))).relativeTo(ReefFace.Two.redPose).getY();

      coralStationMiddleRightIntermediate =
          new Pose2d(Meters.of(12.72), Meters.of(5.915), Rotation2d.fromDegrees(50));
      coralStationMiddleLeftIntermediate =
          new Pose2d(Meters.of(12.72), Meters.of(2.085), Rotation2d.fromDegrees(-50));

    } else {
      coralStationLeft =
          fieldLayout.getTagPose(13).get().toPose2d().transformBy(coralStationTransform);
      coralStationRight =
          fieldLayout.getTagPose(12).get().toPose2d().transformBy(coralStationTransform);
      reefCenter = new Pose2d(Inches.of(176.75), Inches.of(158.5), Rotation2d.fromDegrees(30));
      coralStationRightIntermediate = new Pose2d(Meters.of(5.282), Meters.of(2.5), Rotation2d.kPi);
      coralStationLeftIntermediate = new Pose2d(Meters.of(5.282), Meters.of(6.5), Rotation2d.kPi);

      coralStationMiddleLeftIntermediate =
          new Pose2d(Meters.of(6), Meters.of(6), Rotation2d.fromDegrees(50));
      coralStationMiddleRightIntermediate =
          new Pose2d(Meters.of(6), Meters.of(2), Rotation2d.fromDegrees(-50));
    }

    // Log all Pose2D of relevant positions in AdvantageScope
    Logger.recordOutput("FieldMap/ReefOneRight", getPolePose(ReefFace.One, ReefPole.Right));
    Logger.recordOutput("FieldMap/ReefOneLeft", getPolePose(ReefFace.One, ReefPole.Left));
    Logger.recordOutput("FieldMap/ReefTwoRight", getPolePose(ReefFace.Two, ReefPole.Right));
    Logger.recordOutput("FieldMap/ReefTwoLeft", getPolePose(ReefFace.Two, ReefPole.Left));
    Logger.recordOutput("FieldMap/ReefThreeRight", getPolePose(ReefFace.Three, ReefPole.Right));
    Logger.recordOutput("FieldMap/ReefThreeLeft", getPolePose(ReefFace.Three, ReefPole.Left));
    Logger.recordOutput("FieldMap/ReefFourRight", getPolePose(ReefFace.Four, ReefPole.Right));
    Logger.recordOutput("FieldMap/ReefFourLeft", getPolePose(ReefFace.Four, ReefPole.Left));
    Logger.recordOutput("FieldMap/ReefFiveRight", getPolePose(ReefFace.Five, ReefPole.Right));
    Logger.recordOutput("FieldMap/ReefFiveLeft", getPolePose(ReefFace.Five, ReefPole.Left));
    Logger.recordOutput("FieldMap/ReefSixRight", getPolePose(ReefFace.Six, ReefPole.Right));
    Logger.recordOutput("FieldMap/ReefSixLeft", getPolePose(ReefFace.Six, ReefPole.Left));
    Logger.recordOutput(
        "FieldMap/CoralStationRightMiddleIntermediate", coralStationMiddleRightIntermediate);
    Logger.recordOutput("FieldMap/CoralStationRightIntermediate", coralStationRightIntermediate);
    Logger.recordOutput("FieldMap/CoralStationRight", coralStationRight);
    Logger.recordOutput(
        "FieldMap/CoralStationLeftMiddleIntermediate", coralStationMiddleLeftIntermediate);
    Logger.recordOutput("FieldMap/CoralStationLeftIntermediate", coralStationLeftIntermediate);
    Logger.recordOutput("FieldMap/CoralStationLeft", coralStationLeft);
  }

  // Clockwise from farthest face from the driver
  public enum ReefFace {
    One(10, 21),
    Two(9, 22),
    Three(8, 17),
    Four(7, 18),
    Five(6, 19),
    Six(11, 20);

    public final Pose2d redPose;
    public final Pose2d bluePose;

    private ReefFace(int redApriltag, int blueAprilTag) {
      this.redPose = fieldLayout.getTagPose(redApriltag).get().toPose2d();
      this.bluePose = fieldLayout.getTagPose(blueAprilTag).get().toPose2d();
    }
  }

  public enum ReefPole {
    Right(rightReefTransform),
    FarRight(farRightReefTransform),
    Left(leftReefTransform),
    FarLeft(farLeftReefTransform),
    L2L3Left(L2L3LeftReefTransform),
    L2L3Right(L2L3RightReefTransform);

    public final Transform2d reefPoleTransform;

    private ReefPole(Transform2d reefPoleTransform) {
      this.reefPoleTransform = reefPoleTransform;
    }
  }

  public Pose2d getPolePose(ReefFace face, ReefPole pole) {
    if (alliance == Alliance.Blue) {
      return face.bluePose.transformBy(pole.reefPoleTransform);
    } else {
      return face.redPose.transformBy(pole.reefPoleTransform);
    }
  }

  public Pose2d getAlgaeFarPose(ReefFace face) {
    if (alliance == Alliance.Blue) {
      return face.bluePose.transformBy(farAlgaeTransform);
    } else {
      return face.redPose.transformBy(farAlgaeTransform);
    }
  }

  public Pose2d getAlgaeClosePose(ReefFace face) {
    if (alliance == Alliance.Blue) {
      return face.bluePose.transformBy(closeAlgaeTransform);
    } else {
      return face.redPose.transformBy(closeAlgaeTransform);
    }
  }

  public ReefFace getLockedReefFace(Pose2d currentPose) {
    Pose2d reefRelativePose = currentPose.relativeTo(reefCenter);
    angleToReef = Math.atan2(reefRelativePose.getY(), reefRelativePose.getX());
    Logger.recordOutput("AngleToReef", angleToReef);
    if (angleToReef > (2 * Math.PI) / 3) {
      return ReefFace.Four;
      // return ReefFace.One;
    } else if (angleToReef > (Math.PI) / 3) {
      return ReefFace.Five;
      // return ReefFace.Two;
    } else if (angleToReef > 0) {
      return ReefFace.Six;
      // return ReefFace.Three;
    } else if (angleToReef > (-Math.PI / 3)) {
      return ReefFace.One;
      // return ReefFace.Four;
    } else if (angleToReef > ((-Math.PI * 2) / 3)) {
      return ReefFace.Two;
      // return ReefFace.Five;
    } else {
      return ReefFace.Three;
      // return ReefFace.Six;
    }
  }

  public Rotation2d getLockedReefFaceAngle(ReefFace reefFace) {

    if (reefFace == ReefFace.One) {
      return Rotation2d.fromDegrees(180);
    } else if (reefFace == ReefFace.Two) {
      return Rotation2d.fromDegrees(120);
    } else if (reefFace == ReefFace.Three) {
      return Rotation2d.fromDegrees(60);
    } else if (reefFace == ReefFace.Four) {
      return Rotation2d.fromDegrees(0);
    } else if (reefFace == ReefFace.Five) {
      return Rotation2d.fromDegrees(-60);
    } else {
      return Rotation2d.fromDegrees(-120);
    }

    // if (reefFace == ReefFace.One) {
    //     return Rotation2d.fromDegrees(0);
    //   } else if (reefFace == ReefFace.Two) {
    //     return Rotation2d.fromDegrees(-60);
    //   } else if (reefFace == ReefFace.Three) {
    //     return Rotation2d.fromDegrees(-120);
    //   } else if (reefFace == ReefFace.Four) {
    //     return Rotation2d.fromDegrees(180);
    //   } else if (reefFace == ReefFace.Five) {
    //     return Rotation2d.fromDegrees(120);
    //   } else {
    //     return Rotation2d.fromDegrees(60);
    //   }
  }

  public double getLeftPolePitch() {
    return leftPolePitch;
  }

  public double getLeftPoleYaw() {
    return leftPoleYaw;
  }

  public double getRightPolePitch() {
    return rightPolePitch;
  }

  public double getRightPoleYaw() {
    return rightPoleYaw;
  }

  public void updateCurrentReefAngle(Pose2d currentPose) {
    Pose2d tagPose = null;
    if (alliance == Alliance.Blue) {
      tagPose = getLockedReefFace(currentPose).bluePose;
    } else {
      tagPose = getLockedReefFace(currentPose).redPose;
    }
    Pose2d tagRelativePose = currentPose.relativeTo(tagPose);
    angleToTag = Math.atan2(tagRelativePose.getY(), tagRelativePose.getX());
  }

  public double getCurrentAngleToTag() {
    return angleToTag;
  }

  public void periodic() {
    Logger.recordOutput("AngleToReef", angleToReef);
    // Logger.recordOutput("LeftRelativeX", leftrelativeX);
    // Logger.recordOutput("LeftRelativeY",leftRelativeY);

    // Logger.recordOutput("RightRelativeX", rightrelativeX);
    // Logger.recordOutput("RightRelativeY", rightRelativeY);

  }
}
