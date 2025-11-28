package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import java.util.*;
import java.util.function.Supplier;

public class PathToCommand extends Command {
  // Required for command, and a holonomic pose supplier
  Drivetrain drivetrain;
  HolonomicPose currentPose;
  HolonomicPose endPose;
  Supplier<HolonomicPose>[] pathPoseSuppliers;
  Supplier<LinearVelocity> endVelocitySupplier;
  // Default end velocity
  LinearVelocity endVelocity = MetersPerSecond.of(0);
  PathPlannerPath path;
  Command pathCommand;
  boolean endDeferred = false;
  boolean velocityDeferred = false;

  Distance xTolerance = Meters.of(0.05);
  Distance yTolerance = Meters.of(0.05);
  Distance tolerance = Meters.of(Math.hypot(xTolerance.in(Meters), yTolerance.in(Meters)));
  // May need to be tuned
  Angle rotTolerance = Radians.of(Math.PI / 36);

  /**
   * Default constructor, use this to create a command which will always go to the constructed endpose.
   * @param endPose Ideal endposition, cannot be changed after command is created
   * @param dt Start pose supplier, Command Requirement
   */
  public PathToCommand(Drivetrain dt, HolonomicPose endPose) {
    this.drivetrain = dt;
    this.endPose = endPose;

    addRequirements(drivetrain);
  }

  /**
   * Path command constructor, use to set a non zero end velocity
   * @param endPose Ideal endposition, cannot be changed after command is created
   * @param endVelocity Ideal (non-zero) end velocity
   * @param dt Start pose supplier, Command Requirement
   */
  public PathToCommand(Drivetrain dt, HolonomicPose endPose, LinearVelocity endVelocity) {
    this.drivetrain = dt;
    this.endPose = endPose;
    this.endVelocity = endVelocity;

    addRequirements(drivetrain);
  }

  /**
   * Constructs a path command with a deferred end pose (allows changing intended end target)
   * @param poseSuppliers Suppliers of path poses, in order from first to last. Do not supply starting pose,
   * as this is handled by passing drivetrain
   * @param dt Start pose supplier, Command Requirement
   */
  @SuppressWarnings("unchecked")
  public PathToCommand(Drivetrain dt, Supplier<HolonomicPose>... poseSuppliers) {
    this.pathPoseSuppliers = poseSuppliers;
    this.drivetrain = dt;
    this.endDeferred = true;
    addRequirements(drivetrain);
  }

  /**
   *
   * @param dt
   * @param endVelocitySupplier
   * @param poseSuppliers Suppliers of path poses, in order from first to last
   */
  @SuppressWarnings("unchecked")
  public PathToCommand(
      Drivetrain dt,
      Supplier<LinearVelocity> endVelocitySupplier,
      Supplier<HolonomicPose>... poseSuppliers) {
    this.pathPoseSuppliers = poseSuppliers;
    this.drivetrain = dt;
    this.endDeferred = true;
    this.velocityDeferred = true;
    addRequirements(drivetrain);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    try {
      ArrayList<Pose2d> poses = new ArrayList<>();

      if (endDeferred) {
        if (pathPoseSuppliers == null || pathPoseSuppliers.length == 0) {
          throw new RuntimeException("Deferred path requested but no pose suppliers given.");
        }

        // add poses
        for (int i = 0; i < pathPoseSuppliers.length; i++) {
          poses.add(pathPoseSuppliers[i].get().getPose());
        }

        // initialize endpose
        endPose = pathPoseSuppliers[pathPoseSuppliers.length - 1].get();

      } else {
        if (endPose == null) {
          throw new RuntimeException("Non-deferred PathToCommand created without an end pose.");
        }
        poses.add(endPose.getPose());
      }

      this.currentPose = new HolonomicPose(drivetrain);

      if (velocityDeferred && endVelocitySupplier != null) {
        this.endVelocity = endVelocitySupplier.get();
      }

      // build path
      path = getPath(endVelocity, poses);

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
              drivetrain)
          .withName("Path Command");

    } catch (Exception e) {
      DriverStation.reportError(
          "Could not create go to command. " + e.getMessage(), e.getStackTrace());
      pathCommand = Commands.none();
    }

    pathCommand.initialize();
  }

  /** The main body of a command. Called repeatedly while the command is scheduled. */
  public void execute() {
    // Recheck current pose every loop cycle
    currentPose = new HolonomicPose(drivetrain);
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
    // System.out.println("ending");
    pathCommand.end(interrupted);
    drivetrain.xWheels();
  }

  /**
   * Drivetrain within tolerance
   */
  @Override
  public boolean isFinished() {
    return currentPose.isNear(endPose, tolerance, rotTolerance);
  }
  /**
   * Sets end velocity. This must be done before the command is initialized by the scheduler
   * @param endVelocity
   */
  public void setEndVelocity(LinearVelocity endVelocity) {
    this.endVelocity = endVelocity;
  }

  public void setEndPose(HolonomicPose endPose) {
    this.endPose = endPose;
  }

  /**
   * Creates a simple pathplanner path from a list of poses
   * @param endVelocity Target end velocity
   * @param poses Waypoints
   * @return
   */
  private PathPlannerPath getPath(LinearVelocity endVelocity, List<Pose2d> poses) {

    double startingVelocity = Math.hypot(
        drivetrain.getState().Speeds.vyMetersPerSecond,
        drivetrain.getState().Speeds.vxMetersPerSecond);

    // Waypoints generated from current, any intermediate poses, and endpose
    var all_poses = new ArrayList<Pose2d>(1 + poses.size());
    all_poses.add(drivetrain.getPose());
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
