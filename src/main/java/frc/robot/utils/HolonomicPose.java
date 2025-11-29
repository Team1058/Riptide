package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class HolonomicPose {
  private Pose2d pose;
  private Rotation2d heading; // Direction of travel

  /**
   * Defines a Pose2d with heading, useful for swerve drives where direction of travel
   * is not necessarily the direction the robot is facing
   * @param pose Pose2d of final position, with rotation indicating direction of travel
   * @param heading Heading of final position, indicating direction the robot should be facing
   */
  public HolonomicPose(Pose2d pose, Rotation2d heading) {
    this.pose = pose;
    this.heading = heading;
    initializeLogging();
  }

  public HolonomicPose(Drivetrain dt) {
    double x = dt.getCurrentSpeeds().vxMetersPerSecond;
    double y = dt.getCurrentSpeeds().vyMetersPerSecond;

    Rotation2d directionOfTravel = (Math.hypot(x, y) > 1e-4)
        ? new Rotation2d(Math.atan2(y, x))
        : dt.getPose().getRotation();

    this.pose = new Pose2d(dt.getPose().getTranslation(), directionOfTravel);
    this.heading = dt.getPose().getRotation();
    initializeLogging();
  }

  public HolonomicPose(Translation2d translation, Rotation2d rotation, Rotation2d heading) {
    this.pose = new Pose2d(translation, rotation);
    this.heading = heading;
    initializeLogging();
  }
  /**
   * Defines a Pose2d with heading and direction of travel
   * This constructor asssumes direction of travel and heading to be the same.
   * @param pose
   */
  public HolonomicPose(Pose2d pose) {
    this.pose = pose;
    this.heading = pose.getRotation();
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

  public void update(Pose2d pose, Rotation2d heading) {
    this.pose = pose;
    this.heading = heading;
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

  /**
   * Checks if holonomic pose is within tolerance positionally and rotationally
   * @param otherPose Pose to check against
   * @param tolerance Position tolerance
   * @param rotTolerance Rotation tolerance
   * @return boolean
   */
  public boolean isNear(HolonomicPose otherPose, Distance tolerance, Angle rotTolerance) {
    return getTranslation().getDistance(otherPose.getTranslation()) < tolerance.in(Meters)
        && Math.abs(getHeading().minus(otherPose.getHeading()).getRadians()) < rotTolerance.in(Radians);
  }

  private void initializeLogging() {
    Logger.recordOutput("/HolonomicPose", this.pose);
  }

  @Override
  public String toString() {
    return String.format(
        "PathPose(x=%.2f, y=%.2f, rot=%.2f°, heading=%.2f°)",
        pose.getX(), pose.getY(), pose.getRotation().getDegrees(), heading.getDegrees());
  }
}
