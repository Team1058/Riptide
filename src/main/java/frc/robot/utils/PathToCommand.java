package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
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
  // Initialize as empty array
  ArrayList<Supplier<HolonomicPose>> poseSuppliers = new ArrayList<>();
  Supplier<LinearVelocity> endVelocitySupplier = () -> MetersPerSecond.of(0);
  PathPlannerPath path;
  Command pathCommand;
  PathConstraints pathConstraints;

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
    poseSuppliers.add(() -> endPose);
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
    poseSuppliers.add(() -> endPose);
    endVelocitySupplier = () -> endVelocity;
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
    // iterate over pose suppliers and append each in order passed
    for (int i = 0; i < poseSuppliers.length; i++) {
      this.poseSuppliers.add(poseSuppliers[i]);
    }
    this.drivetrain = dt;
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

    for (int i = 0; i < poseSuppliers.length; i++) {
      this.poseSuppliers.add(poseSuppliers[i]);
    }
    this.endVelocitySupplier = endVelocitySupplier;
    this.drivetrain = dt;
    addRequirements(drivetrain);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    try {
      ArrayList<Pose2d> poses = new ArrayList<>();

      poseSuppliers.forEach((supplier) -> poses.add(supplier.get().getPose()));

      this.currentPose = new HolonomicPose(drivetrain);
      this.endPose = poseSuppliers.get(poseSuppliers.size() - 1).get();
      if (pathConstraints == null) {
        pathConstraints = drivetrain.pathConstraints;
      }
      // build path
      path = getPath(endVelocitySupplier.get(), poses);

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
    // Is this too expensive? Is there a better way of doing this?
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
   * Sets the max speed of the pathcommand
   * @param speed
   * @return
   */
  public PathToCommand withMaxSpeed(LinearVelocity speed) {
    if (pathConstraints != null) {
      pathConstraints = new PathConstraints(
          speed,
          pathConstraints.maxAcceleration(),
          pathConstraints.maxAngularVelocity(),
          pathConstraints.maxAngularAcceleration());
    } else {
      // these should be the drivetrains default path constraints;
      pathConstraints = new PathConstraints(
          speed,
          MetersPerSecondPerSecond.of(3.0),
          drivetrain.getMaxAngularVelocity(),
          RadiansPerSecondPerSecond.of(4 * Math.PI));
    }
    return this;
  }

  /**
   * Set path constraints
   * @param constraints
   * @return itself for method chaining
   */
  public PathToCommand withPathConstraints(PathConstraints constraints) {
    pathConstraints = constraints;
    return this;
  }

  /**
   * Adds a new waypoint to the front of the path
   * @param waypoint Supplier of a HolonomicPose
   * @return itself for method chaining
   */
  public PathToCommand withNewWaypoint(Supplier<HolonomicPose> waypoint) {
    ArrayList<Supplier<HolonomicPose>> temp = new ArrayList<>(1 + poseSuppliers.size());
    temp.add(waypoint);
    temp.addAll(poseSuppliers);
    return this;
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
        pathConstraints,
        new IdealStartingState(startingVelocity, currentPose.getHeading()),
        new GoalEndState(endVelocity, endPose.getHeading()));
    path.preventFlipping = true;
    return path;
  }
}
