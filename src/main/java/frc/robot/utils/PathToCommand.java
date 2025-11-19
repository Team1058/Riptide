package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import java.util.*;

public class PathToCommand extends Command {
  Drivetrain drivetrain;
  HolonomicPose currentPose;
  HolonomicPose endPose;
  LinearVelocity endVelocity;
  PathPlannerPath path;
  Command pathCommand;
  boolean running;

  Distance xTolerance = Meters.of(0.05);
  Distance yTolerance = Meters.of(0.05);
  Translation2d tolerance = new Translation2d(xTolerance, yTolerance);

  public PathToCommand(Drivetrain drivetrain, HolonomicPose endPose, LinearVelocity endVelocity) {
    this.drivetrain = drivetrain;
    this.endPose = endPose;
    this.endVelocity = endVelocity;

    this.currentPose = new HolonomicPose(drivetrain);

    addRequirements(drivetrain);
  }

  public PathToCommand(Drivetrain drivetrain, Pose2d endPose, Rotation2d endHeading) {
    System.out.println("creating");
    this.drivetrain = drivetrain;
    this.endPose = new HolonomicPose(endPose, endHeading);

    addRequirements(drivetrain);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  public void initialize() {
    try {
      // this is not printing, making me think the command is never being initialized
      System.out.println("initializing");
      // Use final pose getter from field map
      path = getPath(endVelocity, List.of(currentPose.getPose(), endPose.getPose()));
      this.pathCommand = new FollowPathCommand(
          path,
          drivetrain::getPose,
          () -> drivetrain.getCurrentSpeeds(),
          (speeds, feedforwards) -> drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds()
              .withSpeeds(speeds)
              .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
              .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          drivetrain.ppDriveController,
          drivetrain.getPPConfig(),
          () -> false,
          drivetrain);
    } catch (Exception e) {
      DriverStation.reportError(
          "Could not create go to command. " + e.getMessage(), e.getStackTrace());
      pathCommand = Commands.none();
    }
  }

  /** The main body of a command. Called repeatedly while the command is scheduled. */
  public void execute() {

    // this prints when execute called
    System.out.println("executing");
    pathCommand.execute();
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally, or
   * when it interrupted/canceled.
   *
   * <p>Do not schedule commands here that share requirements with this command. Use {@link
   * #andThen(Command...)} instead.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  public void end(boolean interrupted) {
    System.out.println("ending");
    pathCommand.end(interrupted);
    drivetrain.xWheels();
  }

  /**
   * This command should be interrupted
   */
  public boolean isFinished() {
    return false;
  }

  private PathPlannerPath getPath(LinearVelocity endVelocity, List<Pose2d> poses) {

    double startingVelocity = Math.hypot(
        drivetrain.getState().Speeds.vyMetersPerSecond,
        drivetrain.getState().Speeds.vxMetersPerSecond);

    // Waypoints generated from current, any intermediate poses, and endpose
    var all_poses = new ArrayList<Pose2d>(1 + poses.size());
    all_poses.add(currentPose.getPose());
    all_poses.addAll(poses);
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(all_poses);

    path = new PathPlannerPath(
        waypoints,
        drivetrain.pathConstraints,
        new IdealStartingState(startingVelocity, currentPose.getHeading()),
        new GoalEndState(endVelocity, endPose.getHeading()));
    path.preventFlipping = true;
    return path;
  }
}
