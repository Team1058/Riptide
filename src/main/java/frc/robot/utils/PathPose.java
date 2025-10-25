package frc.robot.utils;

import edu.wpi.first.math.geometry.*;


public class PathPose extends Pose2d {

  private Rotation2d heading;
  private Pose2d pose;
  private final Transform2d flip180deg = new Transform2d(0, 0, Rotation2d.k180deg);
  /**
   * This class is for making paths. When making a path using PathPlanner,
   * the heading is the final rotation of the robot and the rotation2d inside the array of poses is the direction of travel
   * @param pose
   * @param directionOfTravelIsSameAsHeading
   */
  public PathPose(Pose2d pose, boolean directionOfTravelIsSameAsHeading) {
    this.pose = pose;
    this.heading = directionOfTravelIsSameAsHeading ?  pose.getRotation() : pose.getRotation().rotateBy(Rotation2d.k180deg);
  }

  public Rotation2d getHeading() {
    return heading;
  }



}
