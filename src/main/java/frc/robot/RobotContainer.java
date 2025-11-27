package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.subsystems.FieldMap.ReefFace;
import frc.robot.subsystems.FieldMap.ReefPole;
import frc.robot.utils.HolonomicPose;
import frc.robot.utils.PathToCommand;
import java.util.Set;

public class RobotContainer {
  Climber climber;
  Shooter shooter;
  Elevator elevator;
  Leds leds;

  RobotConfig robotConfig;
  int driverPort = 0;
  int operatorPort = 1;
  Controllers controllers;
  Drivetrain drivetrain;
  Vision vision;

  CommandXboxController operatorController;
  CommandXboxController driveController;

  FieldMap fieldMap;
  SwerveRequest.FieldCentric drive;
  SwerveRequest.FieldCentricFacingAngle reefLock;
  SwerveRequest.SwerveDriveBrake brake;

  private Command driveToNearestLeftPole;
  private Command driveToNearestRightPole;

  private Command driveToLeftCoralStation;
  private Command driveToRightCoralStation;

  private Command driveSlowlyToNearestLeftPole;
  private Command driveSlowlyToNearestRightPole;

  private Command driveNearLeftPole;
  private Command driveNearRightPole;

  private Command driveNearLeftPoleForLowerL;
  private Command driveNearRightPoleForLowerL;

  private Command autoScoreL2Left;
  private Command autoScoreL3Left;
  private Command autoScoreL4Left;
  private Command autoScoreL2Right;
  private Command autoScoreL3Right;
  private Command autoScoreL4Right;

  private double moveIntoReefSpeed = 1;
  private boolean reefLockEnabled = false;

  SendableChooser<Command> autoChooser;
  private ShuffleboardTab autosTab;

  private Command moveElevatorToAlgae1AndIntakeAlgae;
  private Command moveElevatorToAlgae2AndIntakeAlgae;

  public RobotContainer() {

    String serialNumber = System.getenv("serialnum");
    RoboRio roboRio = RoboRio.lookupBySerialNumber(serialNumber);
    robotConfig = RobotConfig.lookupConfig(roboRio);
    driveController = new CommandXboxController(robotConfig.driverControllerPort);
    operatorController = new CommandXboxController(robotConfig.operatorControllerPort);
    climber = new Climber(robotConfig.climberConfig);
    elevator = new Elevator(robotConfig.elevatorConfig);
    shooter = new Shooter(robotConfig.shooterConfig);
    controllers = new Controllers(driveController, operatorController);
    leds = new Leds(elevator);
    fieldMap = new FieldMap(Alliance.Red);
    initDrivetrain(robotConfig.drivetrainConfig);
    initVision(robotConfig.visionConfig);
    initAutoScoreCommands();
    initNamedCommands();
    configureDriverBindings(robotConfig.drivetrainConfig);
    configureOperatorBindings();
    initAuto();
  }

  private void configureOperatorBindings() {
    if (robotConfig.hasClimber) {
      operatorController.leftTrigger(0.5).onTrue(climber.deployClimberCommand());
      operatorController
          .rightTrigger(.1)
          .and(() -> climber.leaderMotor.getAbsoluteEncoder().getPosition() >= 0.2)
          .whileTrue(elevator
              .setRequestedPositionCommand(elevator.LEVEL1::getAndUpdate)
              .andThen(elevator.goToRequestedPositionCommand())
              .alongWith(shooter.openSesameCommand())
              .andThen(climber.manualClimbCommand(operatorController::getRightTriggerAxis)));
    }

    operatorController.rightStick().whileTrue(shooter.unIntakeCommand());

    operatorController
        .start()
        .debounce(.25)
        .and(operatorController.back().negate())
        .onTrue(elevator.resetElevatorCommand());

    if (robotConfig.hasShooter) {
      operatorController
          .a()
          .whileTrue(elevator
              .setRequestedPositionCommand(elevator.LEVELHP::getAndUpdate)
              .andThen(elevator.goToRequestedPositionCommand())
              .andThen(shooter.intakeCoralCommand()));

      // Going to Coral positions commands
      operatorController
          .povUp()
          .and(() -> shooter.coralDetectedByEitherSensor()
              || operatorController.rightBumper().getAsBoolean())
          .onTrue(elevator.setRequestedPositionCommand(elevator.LEVEL1::getAndUpdate));
      operatorController
          .povRight()
          .and(() -> shooter.coralDetectedByEitherSensor()
              || operatorController.rightBumper().getAsBoolean())
          .onTrue(elevator.setRequestedPositionCommand(elevator.LEVEL2::getAndUpdate));
      operatorController
          .povDown()
          .and(() -> shooter.coralDetectedByEitherSensor()
              || operatorController.rightBumper().getAsBoolean())
          .onTrue(elevator.setRequestedPositionCommand(elevator.LEVEL3::getAndUpdate));
      operatorController
          .povLeft()
          .and(() -> shooter.coralDetectedByEitherSensor()
              || operatorController.rightBumper().getAsBoolean())
          .onTrue(elevator.setRequestedPositionCommand(elevator.LEVEL4::getAndUpdate));

      // Going to the algae positions
      operatorController
          .povUp()
          .and(() -> shooter.coralNotDetectedByEitherSensor()
              || operatorController.leftBumper().getAsBoolean())
          .and(operatorController.rightBumper().negate())
          .whileTrue(elevator
              .setRequestedPositionCommand(elevator.LEVELALGAEFLOOR::getAndUpdate)
              .alongWith(shooter.deployAlgaeHookFloorCommand())
              .andThen(() -> shooter.setSpeed(-1)))
          .toggleOnFalse(shooter
              .retractAlgaeHookCommand()
              .andThen(new WaitCommand(0.5))
              .andThen(() -> shooter.setSpeed(-0.2)));
      operatorController
          .povRight()
          .and(() -> shooter.coralNotDetectedByEitherSensor()
              || operatorController.leftBumper().getAsBoolean())
          .and(operatorController.rightBumper().negate())
          .onTrue(elevator
              .setRequestedPositionCommand(elevator.LEVELALGAEPROC::getAndUpdate)
              .alongWith(shooter.deployAlgaeHookCommand()));

      operatorController
          .povDown()
          .and(() -> shooter.coralNotDetectedByEitherSensor()
              || operatorController.leftBumper().getAsBoolean())
          .and(operatorController.rightBumper().negate())
          .onTrue(elevator.setRequestedPositionCommand(elevator.LEVELALGAE2::getAndUpdate));
      operatorController
          .povLeft()
          .and(() -> shooter.coralNotDetectedByEitherSensor()
              || operatorController.leftBumper().getAsBoolean())
          .and(operatorController.rightBumper().negate())
          .onTrue(elevator
              .setRequestedPositionCommand(elevator.LEVELBARGE::getAndUpdate)
              .andThen(shooter.holdAlgaeHookCommand()));

      operatorController
          .back()
          .and(operatorController.start())
          .onTrue(new RunCommand(() -> swapControllers(), controllers)
              .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
      operatorController
          .b()
          .and(shooter::coralNotDetectedByEitherSensor)
          .whileTrue(elevator
              .setRequestedPositionCommand(elevator.LEVELALGAE1::getAndUpdate)
              .andThen(elevator.goToRequestedPositionCommand())
              .andThen(new WaitUntilCommand(
                  () -> elevator.currentPositionAtTarget(elevator.LEVELALGAE1.getAndUpdate())))
              .andThen(shooter.deployAlgaeHookCommand())
              .andThen(shooter.intakeAlgae()))
          .toggleOnFalse(shooter
              .holdAlgaeHookCommand()
              .andThen(elevator.setRequestedPositionCommand(elevator.LEVEL1::getAndUpdate))
              .andThen(elevator.goToRequestedPositionCommand()));
      operatorController
          .y()
          .and(shooter::coralNotDetectedByEitherSensor)
          .whileTrue(elevator
              .setRequestedPositionCommand(elevator.LEVELALGAE2::getAndUpdate)
              .andThen(elevator.goToRequestedPositionCommand())
              .andThen(new WaitUntilCommand(
                  () -> elevator.currentPositionAtTarget(elevator.LEVELALGAE2.getAndUpdate())))
              .andThen(shooter.deployAlgaeHookCommand())
              .andThen(shooter.intakeAlgae()))
          .toggleOnFalse(shooter
              .holdAlgaeHookCommand()
              .andThen(elevator.setRequestedPositionCommand(elevator.LEVEL1::getAndUpdate))
              .andThen(elevator.goToRequestedPositionCommand()));

      // Shoots the coral and algae out of the shooter and stow algae mech if it is deployed
      operatorController
          .x()
          .and(() -> shooter.coralNotDetectedByEitherSensor())
          .whileTrue(shooter.shootAlgae())
          .toggleOnFalse(shooter.stowAlgaeHookCommand());

      operatorController
          .x()
          .and(() -> !shooter.coralNotDetectedByEitherSensor())
          .whileTrue(shooter.spitOutCoralCommand())
          .toggleOnFalse(shooter.stowAlgaeHookCommand());
    }
  }

  public void teleopInit() {
    shooter.stowAlgaeHookCommand().schedule();
  }

  public void autonomousInit() {
    shooter.stowAlgaeHookCommand().schedule();
  }

  private Trigger reefLockEnabledTrigger() {
    return new Trigger(() -> reefLockEnabled);
  }

  private void configureDriverBindings(Drivetrain.Config config) {
    driveController
        .back()
        .and(driveController.start())
        .onTrue(new RunCommand(() -> swapControllers(), controllers)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // driveController
    //     .back()
    //     .debounce(.25)
    //     .and(driveController.start().negate())
    //     .onTrue(cameraCalibrator.runCameraCalibration());
    driveController
        .start()
        .debounce(.25)
        .and(driveController.back().negate())
        .onTrue(drivetrain.resetPerspective());

    driveController
        .rightTrigger(.5)
        .whileTrue(drivetrain.applyRequest(
            () -> drive
                .withVelocityX(Drivetrain.MAX_LINEAR_SPEED.times(-controllers.getDriverLeftY()
                    * 0.125)) // Drive forward with negative Y (forward)
                .withVelocityY(Drivetrain.MAX_LINEAR_SPEED.times(
                    -controllers.getDriverLeftX() * 0.125)) // Drive left with negative X (left)
                .withRotationalRate(drivetrain
                    .getMaxAngularVelocity()
                    .times(-controllers.getDriverRightX()
                        * 0.125)) // Drive counterclockwise with negative X (left)
            ));

    // driveController.a().toggleOnTrue(new RunCommand(()->reefLockEnabled=!reefLockEnabled));

    driveController.a().whileTrue(drivetrain.applyRequest(() -> reefLock
        .withVelocityX(Drivetrain.MAX_LINEAR_SPEED.times(
            -controllers.getDriverLeftY())) // Drive forward with negative Y (forward)
        .withVelocityY(Drivetrain.MAX_LINEAR_SPEED.times(
            -controllers.getDriverLeftX())) // Drive left with negative X (left)
        .withTargetDirection(
            fieldMap.getLockedReefFaceAngle(fieldMap.getLockedReefFace(drivetrain.getPose())))));

    // reefLockEnabledTrigger().whileTrue(drivetrain.applyRequest(() -> reefLock
    //     .withVelocityX(Drivetrain.MAX_LINEAR_SPEED.times(
    //         -controllers.getDriverLeftY())) // Drive forward with negative Y (forward)
    //     .withVelocityY(Drivetrain.MAX_LINEAR_SPEED.times(
    //         -controllers.getDriverLeftX())) // Drive left with negative X (left)
    //     .withTargetDirection(
    //         fieldMap.getLockedReefFaceAngle(fieldMap.getLockedReefFace(drivetrain.getPose())))));

    driveController
        .x()
        .whileTrue(Commands.defer(
            () -> {
              var reefFace = fieldMap.getLockedReefFace(drivetrain.getPose());
              var farPose = fieldMap.getAlgaeFarPose(reefFace);
              var closePose = fieldMap.getAlgaeClosePose(reefFace);
              if (reefFace == ReefFace.One
                  || reefFace == ReefFace.Three
                  || reefFace == ReefFace.Five) {
                // Low Algae ReefFace
                return CommandUtil.wrappedEventCommand(moveElevatorToAlgae1AndIntakeAlgae)
                    .alongWith(new WaitCommand(0.5)
                        .andThen(drivetrain.makeGoToCommandWithMaxMPS(
                            closePose.getRotation(),
                            MetersPerSecond.of(Math.hypot(
                                drivetrain.getCurrentSpeeds().vxMetersPerSecond,
                                drivetrain.getCurrentSpeeds().vyMetersPerSecond)),
                            MetersPerSecond.of(0),
                            1.0,
                            farPose,
                            closePose)));
              } else {
                // High Algae ReefFace
                return CommandUtil.wrappedEventCommand(moveElevatorToAlgae2AndIntakeAlgae)
                    .alongWith(new WaitCommand(0.5)
                        .andThen(drivetrain.makeGoToCommandWithMaxMPS(
                            closePose.getRotation(),
                            MetersPerSecond.of(Math.hypot(
                                drivetrain.getCurrentSpeeds().vxMetersPerSecond,
                                drivetrain.getCurrentSpeeds().vyMetersPerSecond)),
                            MetersPerSecond.of(0),
                            1.0,
                            farPose,
                            closePose)));
              }
            },
            Set.of(drivetrain)));

    driveController
        .x()
        .onFalse(Commands.defer(
                () -> {
                  var reefFace = fieldMap.getLockedReefFace(drivetrain.getPose());
                  var farPose = fieldMap.getAlgaeFarPose(reefFace);
                  var closePose = fieldMap.getAlgaeClosePose(reefFace);
                  return drivetrain.makeGoToCommandWithMaxMPS(
                      farPose.getRotation(),
                      MetersPerSecond.of(Math.hypot(
                          drivetrain.getCurrentSpeeds().vxMetersPerSecond,
                          drivetrain.getCurrentSpeeds().vyMetersPerSecond)),
                      MetersPerSecond.of(0),
                      1.0,
                      closePose,
                      farPose);
                },
                Set.of(drivetrain))
            .andThen(shooter
                .holdAlgaeHookCommand()
                .andThen(elevator.setRequestedPositionCommand(elevator.LEVEL1::get))));

    driveController.a().and(driveController.leftBumper()).whileTrue(driveToNearestLeftPole);
    driveController.a().and(driveController.rightBumper()).whileTrue(driveToNearestRightPole);

    driveController.y().and(driveController.leftBumper()).whileTrue(driveNearLeftPoleForLowerL);
    driveController.y().and(driveController.rightBumper()).whileTrue(driveNearRightPoleForLowerL);

    driveController
        .b()
        .and(driveController.leftBumper())
        .whileTrue(new PathToCommand(drivetrain, new HolonomicPose(fieldMap.coralStationLeft)));
    driveController
        .b()
        .and(driveController.rightBumper())
        .whileTrue(new PathToCommand(drivetrain, new HolonomicPose(fieldMap.coralStationRight)));

    driveController
        .b()
        .and(driveController.rightBumper().or(driveController.leftBumper()).negate())
        .whileTrue(new PathToCommand(
            drivetrain, new HolonomicPose(fieldMap.coralStationMiddleRightIntermediate)));

    // driveController
    //     .pov(0)
    //     .and(() -> (driveController.leftBumper().getAsBoolean()
    //         || driveController.rightBumper().getAsBoolean()))
    //     .whileTrue(Commands.defer(
    //         () -> {
    //           ReefPole endReefPole =
    //               driveController.rightBumper().getAsBoolean() ? ReefPole.Right : ReefPole.Left;
    //           Pose2d intermediatePose = driveController.rightBumper().getAsBoolean()
    //               ? fieldMap.coralStationMiddleRightIntermediate
    //               : fieldMap.coralStationMiddleLeftIntermediate;
    //           var endPose = fieldMap.getPolePose(ReefFace.One, endReefPole);
    //           return drivetrain
    //               .makeGoToPoseCommand(intermediatePose, intermediatePose.getRotation(), 2.0)
    //               .andThen(drivetrain.makeGoToPoseCommand(
    //                   endPose, endPose.getRotation().rotateBy(Rotation2d.k180deg), 0.0));
    //         },
    //         Set.of(drivetrain)));

    // driveController
    //     .pov(45)
    //     .and(() -> (driveController.leftBumper().getAsBoolean()
    //         || driveController.rightBumper().getAsBoolean()))
    //     .whileTrue(Commands.defer(
    //         () -> {
    //           ReefPole endReefPole =
    //               driveController.rightBumper().getAsBoolean() ? ReefPole.Right : ReefPole.Left;
    //           var endPose = fieldMap.getPolePose(ReefFace.Two, endReefPole);
    //           return drivetrain
    //               .makeGoToPoseCommand(
    //                   fieldMap.coralStationRightIntermediate,
    //                   fieldMap.coralStationRightIntermediate.getRotation(),
    //                   2.0)
    //               .andThen(drivetrain.makeGoToPoseCommand(
    //                   endPose, endPose.getRotation().rotateBy(Rotation2d.k180deg), 0.0));
    //         },
    //         Set.of(drivetrain)));
    // driveController
    //     .pov(135)
    //     .and(() -> (driveController.leftBumper().getAsBoolean()
    //         || driveController.rightBumper().getAsBoolean()))
    //     .whileTrue(Commands.defer(
    //         () -> {
    //           ReefPole endReefPole =
    //               driveController.rightBumper().getAsBoolean() ? ReefPole.Right : ReefPole.Left;
    //           var endPose = fieldMap.getPolePose(ReefFace.Three, endReefPole);
    //           return drivetrain.makeGoToPoseCommand(
    //               endPose, endPose.getRotation().rotateBy(Rotation2d.k180deg), 0.0);
    //         },
    //         Set.of(drivetrain)));

    // driveController
    //     .pov(180)
    //     .and(() -> (driveController.leftBumper().getAsBoolean()
    //         || driveController.rightBumper().getAsBoolean()))
    //     .whileTrue(Commands.defer(
    //         () -> {
    //           ReefPole endReefPole =
    //               driveController.rightBumper().getAsBoolean() ? ReefPole.Right : ReefPole.Left;
    //           var endPose = fieldMap.getPolePose(ReefFace.Four, endReefPole);
    //           return drivetrain.makeGoToPoseCommand(
    //               endPose, endPose.getRotation().rotateBy(Rotation2d.k180deg), 0.0);
    //         },
    //         Set.of(drivetrain)));

    // driveController
    //     .pov(225)
    //     .and(() -> (driveController.leftBumper().getAsBoolean()
    //         || driveController.rightBumper().getAsBoolean()))
    //     .whileTrue(Commands.defer(
    //         () -> {
    //           ReefPole endReefPole =
    //               driveController.rightBumper().getAsBoolean() ? ReefPole.Right : ReefPole.Left;
    //           var endPose = fieldMap.getPolePose(ReefFace.Five, endReefPole);
    //           return drivetrain.makeGoToPoseCommand(
    //               endPose, endPose.getRotation().rotateBy(Rotation2d.k180deg), 0.0);
    //         },
    //         Set.of(drivetrain)));

    // driveController
    //     .pov(315)
    //     .and(() -> (driveController.leftBumper().getAsBoolean()
    //         || driveController.rightBumper().getAsBoolean()))
    //     .whileTrue(Commands.defer(
    //         () -> {
    //           ReefPole endReefPole =
    //               driveController.rightBumper().getAsBoolean() ? ReefPole.Right : ReefPole.Left;
    //           var endPose = fieldMap.getPolePose(ReefFace.Six, endReefPole);
    //           return drivetrain
    //               .makeGoToPoseCommand(
    //                   fieldMap.coralStationLeftIntermediate,
    //                   fieldMap.coralStationLeftIntermediate.getRotation(),
    //                   2.0)
    //               .andThen(drivetrain.makeGoToPoseCommand(
    //                   endPose, endPose.getRotation().rotateBy(Rotation2d.k180deg), 0.0));
    //         },
    //         Set.of(drivetrain)));

    driveController.pov(270).and(driveController.rightBumper()).whileTrue(autoScoreL4Right);
    driveController.pov(180).and(driveController.rightBumper()).whileTrue(autoScoreL3Right);
    driveController.pov(90).and(driveController.rightBumper()).whileTrue(autoScoreL2Right);

    driveController.pov(270).and(driveController.leftBumper()).whileTrue(autoScoreL4Left);
    driveController.pov(180).and(driveController.leftBumper()).whileTrue(autoScoreL3Left);
    driveController.pov(90).and(driveController.leftBumper()).whileTrue(autoScoreL2Left);

    driveController
        .rightTrigger(.5)
        .whileTrue(drivetrain.applyRequest(
            () -> drive
                .withVelocityX(Drivetrain.MAX_LINEAR_SPEED.times(-controllers.getDriverLeftY()
                    * 0.125)) // Drive forward with negative Y (forward)
                .withVelocityY(Drivetrain.MAX_LINEAR_SPEED.times(
                    -controllers.getDriverLeftX() * 0.125)) // Drive left with negative X (left)
                .withRotationalRate(drivetrain
                    .getMaxAngularVelocity()
                    .times(-controllers.getDriverRightX()
                        * 0.125)) // Drive counterclockwise with negative X (left)
            ));
    // operatorController
    //     .b()
    //     .whileTrue(drivetrain.applyRequest(
    //         () -> drive
    //             .withVelocityX(Drivetrain.MAX_LINEAR_SPEED.times(-controllers.getDriverLeftY()
    //                 * 0.125)) // Drive forward with negative Y (forward)
    //             .withVelocityY(Drivetrain.MAX_LINEAR_SPEED.times(
    //                 -controllers.getDriverLeftX() * 0.125)) // Drive left with negative X (left)
    //             .withRotationalRate(drivetrain
    //                 .getMaxAngularVelocity()
    //                 .times(-controllers.getDriverRightX()
    //                     * 0.125)) // Drive counterclockwise with negative X (left)
    //         ));
  }

  private void swapControllers() {
    int tempDriverPort = driverPort;
    driverPort = operatorPort;
    operatorPort = tempDriverPort;
    driveController = new CommandXboxController(driverPort);
    operatorController = new CommandXboxController(operatorPort);
    controllers = new Controllers(driveController, operatorController);
    CommandScheduler.getInstance().disable();
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().getDefaultButtonLoop().clear();
    CommandScheduler.getInstance().enable();
    configureOperatorBindings();
    configureDriverBindings(robotConfig.drivetrainConfig);
  }

  /**
   * Returned command scores and leaves the elevator at the set position
   *
   * @param level Height of elevator
   * @return a Command
   */

  // This returns a sequence so can only be run by itself (no andThens)
  // private Command getScoreL2L3Command(double level) {

  // if (level == elevator.LEVEL4) {
  //     return elevator.setRequestedPositionCommand(level)
  //       .andThen(elevator.goToRequestedPositionCommand())
  //       .andThen(new WaitUntilCommand(() -> elevator.isAtPosition(level)))
  //       .andThen(shooter.timedSpitCoralCommand(0.2));}

  //   return elevator
  //       .setRequestedPositionCommand(level)
  //       .andThen(elevator.goToRequestedPositionCommand())
  //       .andThen(new WaitUntilCommand(() -> elevator.isAtPosition(level)))
  //       .andThen(shooter.timedSpitCoralCommand(0.2));

  // }

  private void initDrivetrain(Drivetrain.Config config) {
    drivetrain = Drivetrain.makeDrivetrain(config);

    if (drivetrain != null) {
      drive = new SwerveRequest.FieldCentric()
          .withDriveRequestType(
              DriveRequestType.Velocity); // Use open-loop control for drive motors

      brake = new SwerveRequest.SwerveDriveBrake();
      reefLock = new FieldCentricFacingAngle()
          .withDeadband(Drivetrain.MAX_LINEAR_SPEED.times(0.1))
          .withDriveRequestType(DriveRequestType.Velocity);
      reefLock.HeadingController.setP(4);
      reefLock.HeadingController.setI(0);
      reefLock.HeadingController.setD(0);
      reefLock.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
      reefLock.HeadingController.setTolerance(0.1);
      reefLock.withRotationalDeadband(drivetrain.getMaxAngularVelocity().times(0.01));
    }

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () -> drive
                .withVelocityX(Drivetrain.MAX_LINEAR_SPEED.times(
                    -controllers.getDriverLeftY())) // Drive forward with negative Y (forward)
                .withVelocityY(Drivetrain.MAX_LINEAR_SPEED.times(
                    -controllers.getDriverLeftX())) // Drive left with negative X (left)
                .withRotationalRate(drivetrain
                    .getMaxAngularVelocity()
                    .times(
                        -controllers
                            .getDriverRightX())) // Drive counterclockwise with negative X (left)
            ));

    driveToNearestLeftPole = Commands.defer(
        () -> {
          var reefFace = fieldMap.getLockedReefFace(drivetrain.getPose());
          var approachPose = fieldMap.getPolePose(reefFace, ReefPole.FarLeft);
          var finalPose = fieldMap.getPolePose(reefFace, ReefPole.Left);
          return drivetrain.makeGoToCommand(
              finalPose.getRotation(), MetersPerSecond.zero(), approachPose, finalPose);
        },
        Set.of(drivetrain));

    // driveToNearestLeftPole = Commands.defer(
    //     () -> new PathToCommand(drivetrain, null, MetersPerSecond.of(0)), Set.of(drivetrain));

    driveToNearestRightPole = Commands.defer(
        () -> {
          var reefFace = fieldMap.getLockedReefFace(drivetrain.getPose());
          var approachPose = fieldMap.getPolePose(reefFace, ReefPole.FarRight);
          var finalPose = fieldMap.getPolePose(reefFace, ReefPole.Right);
          return drivetrain.makeGoToCommand(
              finalPose.getRotation(), MetersPerSecond.zero(), approachPose, finalPose);
        },
        Set.of(drivetrain));

    driveSlowlyToNearestRightPole = Commands.defer(
        () -> {
          var reefFace = fieldMap.getLockedReefFace(drivetrain.getPose());
          var approachPose = fieldMap.getPolePose(reefFace, ReefPole.FarRight);
          var finalPose = fieldMap.getPolePose(reefFace, ReefPole.Right);
          return drivetrain.makeGoToCommandWithMaxMPS(
              finalPose.getRotation(),
              MetersPerSecond.zero(),
              MetersPerSecond.zero(),
              moveIntoReefSpeed,
              approachPose,
              finalPose);
        },
        Set.of(drivetrain));
    driveSlowlyToNearestLeftPole = Commands.defer(
        () -> {
          var reefFace = fieldMap.getLockedReefFace(drivetrain.getPose());
          var approachPose = fieldMap.getPolePose(reefFace, ReefPole.FarLeft);
          var finalPose = fieldMap.getPolePose(reefFace, ReefPole.Left);
          return drivetrain.makeGoToCommandWithMaxMPS(
              finalPose.getRotation(),
              MetersPerSecond.zero(),
              MetersPerSecond.zero(),
              moveIntoReefSpeed,
              approachPose,
              finalPose);
        },
        Set.of(drivetrain));

    driveToLeftCoralStation = Commands.defer(
        () -> {
          var currentPose = drivetrain.getPose();
          var reefFace = fieldMap.getLockedReefFace(currentPose);
          var finalPose = fieldMap.coralStationLeft;
          var finalHeading = fieldMap.coralStationLeft.getRotation();
          finalPose =
              fieldMap.coralStationLeft.transformBy(new Transform2d(0, 0, Rotation2d.k180deg));
          if (reefFace == ReefFace.One || reefFace == ReefFace.Six) {
            return drivetrain.makeGoToCommand(
                finalHeading,
                MetersPerSecond.zero(),
                fieldMap.coralStationMiddleLeftIntermediate,
                finalPose);
          } else if (reefFace == ReefFace.Four || reefFace == ReefFace.Five) {
            return drivetrain.makeGoToCommand(finalHeading, MetersPerSecond.zero(), finalPose);
          }
          return Commands.none();
        },
        Set.of(drivetrain));

    driveToRightCoralStation = Commands.defer(
        () -> {
          var currentPose = drivetrain.getPose();
          var reefFace = fieldMap.getLockedReefFace(currentPose);
          var finalPose = fieldMap.coralStationRight;
          var finalHeading = fieldMap.coralStationRight.getRotation();
          finalPose =
              fieldMap.coralStationRight.transformBy(new Transform2d(0, 0, Rotation2d.k180deg));
          if (reefFace == ReefFace.One || reefFace == ReefFace.Two) {
            return drivetrain.makeGoToCommand(
                finalHeading,
                MetersPerSecond.zero(),
                fieldMap.coralStationRightIntermediate,
                finalPose);
          } else if (reefFace == ReefFace.Three || reefFace == ReefFace.Four) {
            return drivetrain.makeGoToCommand(finalHeading, MetersPerSecond.zero(), finalPose);
          }
          return Commands.none();
        },
        Set.of(drivetrain));

    driveNearLeftPole = Commands.defer(
        () -> {
          var currentPose = drivetrain.getPose();
          var reefFace = fieldMap.getLockedReefFace(currentPose);
          var finalPose = fieldMap.getPolePose(reefFace, ReefPole.FarLeft);
          return drivetrain.makeGoToCommand(
              finalPose.getRotation(), MetersPerSecond.zero(), currentPose, finalPose);
        },
        Set.of(drivetrain));

    driveNearRightPole = Commands.defer(
        () -> {
          var currentPose = drivetrain.getPose();
          var reefFace = fieldMap.getLockedReefFace(currentPose);
          var finalPose = fieldMap.getPolePose(reefFace, ReefPole.FarRight);
          return drivetrain.makeGoToCommand(
              finalPose.getRotation(), MetersPerSecond.zero(), currentPose, finalPose);
        },
        Set.of(drivetrain));

    driveNearLeftPoleForLowerL = Commands.defer(
        () -> {
          var reefFace = fieldMap.getLockedReefFace(drivetrain.getPose());
          var approachPose = fieldMap.getPolePose(reefFace, ReefPole.FarLeft);
          var finalPose = fieldMap.getPolePose(reefFace, ReefPole.L2L3Left);
          return drivetrain.makeGoToCommandWithMaxMPS(
              finalPose.getRotation(),
              MetersPerSecond.zero(),
              MetersPerSecond.zero(),
              moveIntoReefSpeed,
              approachPose,
              finalPose);
        },
        Set.of(drivetrain));

    driveNearRightPoleForLowerL = Commands.defer(
        () -> {
          var reefFace = fieldMap.getLockedReefFace(drivetrain.getPose());
          var approachPose = fieldMap.getPolePose(reefFace, ReefPole.FarRight);
          var finalPose = fieldMap.getPolePose(reefFace, ReefPole.L2L3Right);
          return drivetrain.makeGoToCommandWithMaxMPS(
              finalPose.getRotation(),
              MetersPerSecond.zero(),
              MetersPerSecond.zero(),
              moveIntoReefSpeed,
              approachPose,
              finalPose);
        },
        Set.of(drivetrain));

    //   configureDriverBindings(config);
    //   registerDriverNamedCommands(config);

  }

  private void initVision(Vision.Config config) {
    if (config == null) {
      return;
    }

    vision = new Vision(config);
    if (drivetrain != null) {
      vision.onPoseUpdate((stampedPose) -> {
        drivetrain.addVisionMeasurement(stampedPose.pose(), stampedPose.timestamp());
        fieldMap.updateCurrentReefAngle(stampedPose.pose());
      });
    }
  }

  private void initAutoScoreCommands() {

    // driveNearLeftPole = CommandUtil.wrappedEventCommand(driveNearLeftPole);
    // driveNearRightPole = CommandUtil.wrappedEventCommand(driveNearRightPole);
    // driveSlowlyToNearestLeftPole = CommandUtil.wrappedEventCommand(driveSlowlyToNearestLeftPole);
    // driveSlowlyToNearestRightPole =
    // CommandUtil.wrappedEventCommand(driveSlowlyToNearestRightPole);

    // autoscore l4: Drive near pole (line up in x but not y), deploy elevator, drive in slowly,
    // score, back off, retract

    autoScoreL4Right = Commands.defer(
        () -> CommandUtil.wrappedEventCommand(driveNearRightPole)
            .andThen(elevator.setRequestedPositionCommand(elevator.LEVEL4::getAndUpdate))
            .andThen(elevator.goToRequestedPositionCommand())
            .andThen(CommandUtil.wrappedEventCommand(driveSlowlyToNearestRightPole))
            .andThen(shooter.timedSpitCoralCommand(0.2))
            .andThen(
                elevator.setRequestedPositionCommand(elevator.LEVEL4CORALDISENGAGE::getAndUpdate))
            .andThen(new WaitUntilCommand(
                    () -> elevator.isAtPosition(elevator.LEVEL4CORALDISENGAGE.getAndUpdate()))
                .withTimeout(0.2))
            // (Do we need an elevator up command to prevent collision with pole?)
            .andThen(CommandUtil.wrappedEventCommand(driveNearRightPole))
            .andThen(elevator.setRequestedPositionCommand(elevator.LEVELHP::getAndUpdate))
            .andThen(new WaitUntilCommand(() -> elevator.isBelowPosition(elevator.LEVEL2.get()))),
        Set.of(drivetrain, elevator, shooter));

    autoScoreL4Left = Commands.defer(
            () -> CommandUtil.wrappedEventCommand(driveNearLeftPole)
                .andThen(elevator.setRequestedPositionCommand(elevator.LEVEL4::getAndUpdate))
                .andThen(elevator.goToRequestedPositionCommand())
                .andThen(CommandUtil.wrappedEventCommand(driveSlowlyToNearestLeftPole))
                .andThen(shooter.timedSpitCoralCommand(0.2))
                .andThen(elevator.setRequestedPositionCommand(
                    elevator.LEVEL4CORALDISENGAGE::getAndUpdate))
                .andThen(new WaitUntilCommand(
                        () -> elevator.isAtPosition(elevator.LEVEL4CORALDISENGAGE.getAndUpdate()))
                    .withTimeout(0.2))
                // (Do we need an elevator up command to prevent collision with pole?)
                .andThen(CommandUtil.wrappedEventCommand(driveNearLeftPole))
                .andThen(elevator.setRequestedPositionCommand(elevator.LEVELHP::getAndUpdate))
                .andThen(
                    new WaitUntilCommand(() -> elevator.isBelowPosition(elevator.LEVEL2.get()))),
            Set.of(drivetrain, elevator, shooter))
        .withName("AutoScoreL4Left");

    autoScoreL3Left = Commands.defer(
            () -> CommandUtil.wrappedEventCommand(driveNearLeftPole)
                .andThen(elevator.setRequestedPositionCommand(elevator.LEVEL3::getAndUpdate))
                .andThen(elevator.goToRequestedPositionCommand())
                .andThen(CommandUtil.wrappedEventCommand(driveSlowlyToNearestLeftPole))
                .andThen(shooter.timedSpitCoralCommand(0.2))
                .andThen(CommandUtil.wrappedEventCommand(driveNearLeftPole))
                .andThen(elevator.setRequestedPositionCommand(elevator.LEVELHP::getAndUpdate)),
            Set.of(drivetrain, elevator, shooter))
        .withName("AutoScoreL3Left");

    autoScoreL3Right = Commands.defer(
            () -> CommandUtil.wrappedEventCommand(driveNearRightPole)
                .andThen(elevator.setRequestedPositionCommand(elevator.LEVEL3::getAndUpdate))
                .andThen(elevator.goToRequestedPositionCommand())
                .andThen(CommandUtil.wrappedEventCommand(driveSlowlyToNearestRightPole))
                .andThen(shooter.timedSpitCoralCommand(0.2))
                .andThen(CommandUtil.wrappedEventCommand(driveNearRightPole))
                .andThen(elevator.setRequestedPositionCommand(elevator.LEVELHP::getAndUpdate)),
            Set.of(drivetrain, elevator, shooter))
        .withName("AutoScoreL3Right");

    autoScoreL2Right = Commands.defer(
            () -> CommandUtil.wrappedEventCommand(driveNearRightPole)
                .andThen(elevator.setRequestedPositionCommand(elevator.LEVEL2::getAndUpdate))
                .andThen(elevator.goToRequestedPositionCommand())
                .andThen(CommandUtil.wrappedEventCommand(driveSlowlyToNearestRightPole))
                .andThen(shooter.timedSpitCoralCommand(0.2))
                .andThen(CommandUtil.wrappedEventCommand(driveNearRightPole))
                .andThen(elevator.setRequestedPositionCommand(elevator.LEVELHP::getAndUpdate)),
            Set.of(drivetrain, elevator, shooter))
        .withName("AutoScoreL2Right");

    autoScoreL2Left = Commands.defer(
            () -> CommandUtil.wrappedEventCommand(driveNearLeftPole)
                .andThen(elevator.setRequestedPositionCommand(elevator.LEVEL2::getAndUpdate))
                .andThen(elevator.goToRequestedPositionCommand())
                .andThen(CommandUtil.wrappedEventCommand(driveSlowlyToNearestLeftPole))
                .andThen(shooter.timedSpitCoralCommand(0.2))
                .andThen(CommandUtil.wrappedEventCommand(driveNearLeftPole))
                .andThen(elevator.setRequestedPositionCommand(elevator.LEVELHP::getAndUpdate)),
            Set.of(drivetrain, elevator, shooter))
        .withName("AutoScoreL2Left");

    moveElevatorToAlgae1AndIntakeAlgae = elevator
        .setRequestedPositionCommand(elevator.LEVELALGAE1)
        .andThen(elevator.goToRequestedPositionCommand())
        .andThen(new WaitUntilCommand(
            () -> elevator.currentPositionAtTarget(elevator.LEVELALGAE1.get())))
        .andThen(shooter.deployAlgaeHookCommand())
        .andThen(shooter.intakeAlgae());

    moveElevatorToAlgae2AndIntakeAlgae = elevator
        .setRequestedPositionCommand(elevator.LEVELALGAE2)
        .andThen(elevator.goToRequestedPositionCommand())
        .andThen(new WaitUntilCommand(
            () -> elevator.currentPositionAtTarget(elevator.LEVELALGAE2.get())))
        .andThen(shooter.deployAlgaeHookCommand())
        .andThen(shooter.intakeAlgae());
    // .andThen(elevator.setRequestedPositionCommand(elevator.LEVEL2))
    // .andThen(new WaitUntilCommand(() -> elevator.isAtPosition(elevator.LEVEL2)))
    // .andThen(shooter.timedSpitCoralCommand(0.2))
    // .andThen(CommandUtil.wrappedEventCommand(driveNearLeftPoleForLowerL))
    // .andThen(elevator.setRequestedPositionCommand(elevator.LEVELHP)),
    // Set.of(drivetrain, elevator, shooter));
  }

  public void initNamedCommands() {
    NamedCommands.registerCommand("autoScoreL4Left", autoScoreL4Left);
    NamedCommands.registerCommand("autoScoreL3Left", autoScoreL3Left);
    NamedCommands.registerCommand("autoScoreL2Left", autoScoreL2Left);
    NamedCommands.registerCommand("autoScoreL4Right", autoScoreL4Right);
    NamedCommands.registerCommand("autoScoreL3Right", autoScoreL3Right);
    NamedCommands.registerCommand("autoScoreL2Right", autoScoreL2Right);
    NamedCommands.registerCommand(
        "setRequestedPositionL4", elevator.setRequestedPositionCommand(elevator.LEVEL4));
    NamedCommands.registerCommand(
        "setRequestedPositionL1", elevator.setRequestedPositionCommand(elevator.LEVEL1));
    NamedCommands.registerCommand(
        "setRequestedPositionHP", elevator.setRequestedPositionCommand(elevator.LEVELHP));
    NamedCommands.registerCommand("goToRequestedPosition", elevator.goToRequestedPositionCommand());
    NamedCommands.registerCommand("spitOutCoral", shooter.timedSpitCoralCommand(1));
    NamedCommands.registerCommand("goToLeftHumanPlayer", driveToLeftCoralStation);
    NamedCommands.registerCommand("goToRightHumanPlayer", driveToRightCoralStation);
    NamedCommands.registerCommand("intakeCoral", shooter.intakeCoralCommand());
  }

  public void updateAlliance(Alliance alliance) {
    fieldMap = new FieldMap(alliance);
    vision.updateAlliance(alliance);
  }

  public void initAuto() {
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("None", Commands.none());
    autosTab = Shuffleboard.getTab("Autos");
    autosTab.add("Auto Chooser", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void ledSetPatternsLogic() {
    if (leds != null) {
      leds.applyPatternsToStrips();

      if ((shooter.getCurrentCommand() != null
              && shooter.getCurrentCommand().getName().equals("Coral intake command"))
          || (shooter.getCurrentCommand() != null
              && shooter.getCurrentCommand().getName().equals("Spit Out Coral Command"))) {
        leds.leftPattern = leds.greenBase;
      } else if (shooter.algaeMechDeployed()) {
        leds.leftPattern = leds.blueBlink;
      } else if (climber.getCurrentCommand() != null
          && climber.getCurrentCommand().getName().equals("Manual Climb Command")) {
        leds.leftPattern = leds.whiteBlink;
      } else if (climber.isClimbing) {
        leds.leftPattern = leds.whiteBase;
      } else {
        leds.leftPattern = leds.redOrangeBlinkWithRsl;
      }

      if (RobotController.getCPUTemp() > 85) {
        leds.middlePattern = leds.redSlowBlink;
      } else if (RobotController.getBatteryVoltage() <= 11.8) {
        leds.middlePattern = leds.yellowBase;
      } else if (RobotController.getBrownoutVoltage() >= RobotController.getBatteryVoltage()) {
        leds.middlePattern = leds.brownBase;
      } else if (RobotController.getCommsDisableCount() > 5) {
        leds.middlePattern = leds.blueBase;
      } else {
        leds.middlePattern = leds.greenBase;
      }
      if (shooter.coralNotDetectedByEitherSensor()
              && !operatorController.rightBumper().getAsBoolean()
          || operatorController.leftBumper().getAsBoolean()) {
        leds.rightPattern = leds.blueProgressMaskWithElevator;
      } else {
        leds.rightPattern = leds.redProgressMaskWithElevator;
      }
    }
  }

}
