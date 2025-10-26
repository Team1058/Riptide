package frc.robot.utils;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;

public class PathToCommand extends Command{
  Drivetrain drivetrain;
  Pose2d endPose;
  PathPlannerPath path

  public Command makeGoToCommand(
      Rotation2d endHeading, LinearVelocity endVelocity, List<Pose2d> poses) {
    try {
      // Use final pose getter from field map
      PathPlannerPath path = getPath(endHeading, endVelocity, poses);
      FollowPathCommand pathFollowCommand = new FollowPathCommand(
          path,
          this::getPose,
          this::getCurrentSpeeds,
          (speeds, feedforwards) -> this.setControl(new SwerveRequest.ApplyRobotSpeeds()
              .withSpeeds(speeds)
              .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
              .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          ppDriveController,
          ppConfig,
          () -> {
            return false;
          },
          this);

      return pathFollowCommand.withName("GoTo");
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to create GoTo command" + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
  public PathToCommand(Drivetrain drivetrain, Pose2d endPose) {
    this.drivetrain = drivetrain;
    this.endPose = endPose;

    addRequirements(drivetrain);
  }

  public void initialize() {

  }

}
