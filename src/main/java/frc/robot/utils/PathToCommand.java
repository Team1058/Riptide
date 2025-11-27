package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  Drivetrain drivetrain;
  HolonomicPose currentPose;
  HolonomicPose endPose;
  Supplier<HolonomicPose> startPoseSupplier;
  Supplier<HolonomicPose>[] pathPoseSuppliers;
  Supplier<LinearVelocity> endVelocitySupplier;
  // Default end velocity
  LinearVelocity endVelocity = MetersPerSecond.of(0);
  PathPlannerPath path;
  Command pathCommand;
  boolean running;
  // Used if a startPoseSupplier is used to construct the command
  boolean startDeferred = false;
  boolean endDeferred = false;
  boolean velocityDeferred = false;

  Distance xTolerance = Meters.of(0.05);
  Distance yTolerance = Meters.of(0.05);
  Angle rotTolerance = Radians.of(Math.PI / 36);
  Translation2d tolerance = new Translation2d(xTolerance, yTolerance);

  // Probably never going to use this lol - Ben
  // /**
  //  * Constructs a path command. Use this if you want startpose to be different from drivetrain
  // (for some reason)
  //  * @param startPoseSupplier Supplier of start pose
  //  * @param endPose Supplier of end pose
  //  * @param dt Command Requirement
  //  */
  // public PathToCommand(Drivetrain dt, Supplier<HolonomicPose> startPoseSupplier,
  // Supplier<HolonomicPose> endPoseSupplier) {
  //   drivetrain = dt;
  //   this.startPoseSupplier = startPoseSupplier;
  //   this.endPoseSupplier = endPoseSupplier;
  //   addRequirements(drivetrain);
  // }

  /**
   * Default constructor, use this to create a command which will always go to the endpose.
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
   * @param poseSuppliers Suppliers of path poses, in order from first to last
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
  public void initialize() {
    try {
      // Use final pose getter from field map
      ArrayList<Pose2d> poses;
      if (endDeferred && pathPoseSuppliers != null) {
        poses = new ArrayList<>();
        for (int i = 0; i < pathPoseSuppliers.length; i++) {
          poses.add(pathPoseSuppliers[i].get().getPose());
        }
      } else {
        poses = new ArrayList<>(List.of(this.endPose.getPose()));
      }

      if (startDeferred && startPoseSupplier != null) {
        this.currentPose = startPoseSupplier.get();
      } else {
        this.currentPose = new HolonomicPose(drivetrain);
      }

      if (velocityDeferred && endVelocitySupplier != null) {
        this.endVelocity = endVelocitySupplier.get();
      }

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
   * Drivetrain within tolerance
   */
  public boolean isFinished() {
    return (drivetrain.getPose().minus(endPose.getPose()).getTranslation().getDistance(tolerance)
            < Math.hypot(xTolerance.baseUnitMagnitude(), yTolerance.baseUnitMagnitude()))
        && (drivetrain
            .getRotation3d()
            .toRotation2d()
            .minus(endPose.getHeading())
            .getMeasure()
            .isNear(Radians.of(0), rotTolerance));
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
