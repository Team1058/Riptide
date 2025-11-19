package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drivetrain;

public class HolonomicPose {
  private final Pose2d pose;
  private final Rotation2d heading; // Direction of travel

  /**
   * Defines a Pose2d with heading, useful for swerve drives where direction of travel
   * is not necessarily the direction the robot is facing
   * @param pose Pose2d of final position, with rotation indicating direction of travel
   * @param heading Heading of final position, indicating direction the robot should be facing
   */
  public HolonomicPose(Pose2d pose, Rotation2d heading) {
    this.pose = pose;
    this.heading = heading;
  }

  public HolonomicPose(Drivetrain dt) {
    double x = dt.getCurrentSpeeds().vxMetersPerSecond;
    double y = dt.getCurrentSpeeds().vyMetersPerSecond;

    Rotation2d directionOfTravel = (Math.hypot(x, y) > 1e-4)
        ? new Rotation2d(Math.atan2(y, x))
        : dt.getPose().getRotation();

    this.pose = new Pose2d(dt.getPose().getTranslation(), directionOfTravel);
    this.heading = dt.getPose().getRotation();
  }

  public HolonomicPose(Translation2d translation, Rotation2d rotation, Rotation2d heading) {
    this.pose = new Pose2d(translation, rotation);
    this.heading = heading;
  }

  public Pose2d getPose() {
    return pose;
  }

  public Rotation2d getTravelDirection() {
    return pose.getRotation();
  }

  public Translation2d getTranslation() {
    return pose.getTranslation();
  }

  public Rotation2d getHeading() {
    return heading;
  }

  public HolonomicPose relativeTo(HolonomicPose pose) {
    Pose2d relativePose = getPose().relativeTo(pose.getPose());
    Rotation2d relativeHeading = getHeading().minus(pose.getHeading());

    return new HolonomicPose(relativePose, relativeHeading);
  }

  public HolonomicPose withHeading(Rotation2d heading) {
    return new HolonomicPose(getPose(), heading);
  }

  /**
   * Interpolate between two poses
   * @param end end pose
   * @param t Percentage along path to interpolate (0 -> beginning, 0.5 -> halfway, 1 -> end)
   * @return a Holonomic Pose
   */
  public HolonomicPose interpolate(HolonomicPose end, double t) {
    return new HolonomicPose(pose.interpolate(end.pose, t), heading.interpolate(end.heading, t));
  }

  @Override
  public String toString() {
    return String.format(
        "PathPose(x=%.2f, y=%.2f, rot=%.2f°, heading=%.2f°)",
        pose.getX(), pose.getY(), pose.getRotation().getDegrees(), heading.getDegrees());
  }
}
