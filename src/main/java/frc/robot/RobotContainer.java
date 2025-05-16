package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  Climber climber;
  Shooter shooter;
  Elevator elevator;
  RobotConfig robotConfig;
  Drivetrain drivetrain;

  CommandXboxController operatorController;
  CommandXboxController driveController;
  
  SwerveRequest.FieldCentric drive;
  SwerveRequest.FieldCentricFacingAngle reefLock;   

  public RobotContainer() {
    driveController = new CommandXboxController(robotConfig.driverControllerPort);
    operatorController = new CommandXboxController(robotConfig.operatorControllerPort);

    String serialNumber = System.getenv("serialnum");
    RoboRio roboRio = RoboRio.lookupBySerialNumber(serialNumber);
    robotConfig = RobotConfig.lookupConfig(roboRio);
    climber = new Climber(robotConfig.climberConfig);
    elevator = new Elevator(robotConfig.elevatorConfig);
    shooter = new Shooter(robotConfig.shooterConfig);
    drivetrain = new Drivetrain(robotConfig.drivetrainConfig);

    configureBindings();
  }


  private void configureBindings() {

    //Drivetrain Bindings
 driveController
        .back()
        .debounce(.25)
        .and(driveController.start().negate())
        .onTrue(cameraCalibrator.runCameraCalibration());
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

    driveController
        .a()
        .and(driveController
            .rightBumper()
            .negate()
            .and(driveController.leftBumper().negate()))
        .whileTrue(drivetrain.applyRequest(() -> reefLock
            .withVelocityX(Drivetrain.MAX_LINEAR_SPEED.times(
                -controllers.getDriverLeftY())) // Drive forward with negative Y (forward)
            .withVelocityY(Drivetrain.MAX_LINEAR_SPEED.times(
                -controllers.getDriverLeftX())) // Drive left with negative X (left)
            .withTargetDirection(fieldMap.getLockedReefFaceAngle(
                fieldMap.getLockedReefFace(drivetrain.getPose())))));


    if (robotConfig.hasClimber) {
      operatorController
          .leftTrigger(0.5)
          .onTrue(climber
              .deployClimberCommand()
              .alongWith(elevator.setRequestedPositionCommand(elevator.LEVEL2))
              .andThen(elevator.goToRequestedPositionCommand()));
    operatorController
    .rightTrigger(.1)
    .whileTrue((elevator.setRequestedPositionCommand(elevator.LEVEL1))
        .andThen(elevator.goToRequestedPositionCommand())
        .andThen(climber.manualClimbCommand(operatorController::getRightTriggerAxis)));
      }    
      
      operatorController
      .start()
      .debounce(.25)
      .and(operatorController.back().negate())
      .onTrue(elevator.resetElevatorCommand());
      
      if (robotConfig.hasShooter) {
        operatorController.a().whileTrue(elevator.setRequestedPositionCommand(elevator.LEVELHP).andThen(elevator.goToRequestedPositionCommand().andThen(shooter.intakeCoralCommand())));
        
        // Going to Coral positions commands
        operatorController
            .povUp()
            .and(shooter::coralDetectedByEitherSensor)
            .onTrue(elevator.setRequestedPositionCommand(elevator.LEVEL1));
        operatorController
            .povRight()
            .and(shooter::coralDetectedByEitherSensor)
            .onTrue(elevator.setRequestedPositionCommand(elevator.LEVEL2));
        operatorController
            .povDown()
            .and(shooter::coralDetectedByEitherSensor)
            .onTrue(elevator.setRequestedPositionCommand(elevator.LEVEL3));
        operatorController
            .povLeft()
            .and(shooter::coralDetectedByEitherSensor)
            .onTrue(elevator.setRequestedPositionCommand(elevator.LEVEL4));
            
        // Going to the algae positions
        operatorController
            .povUp()
            .and(() -> shooter.coralNotDetectedByEitherSensor()
                || operatorController.leftBumper().getAsBoolean())
            .onTrue(elevator
                .setRequestedPositionCommand(elevator.LEVELALGAELOLLIPOP)
                .alongWith(shooter.deployAlgaeHookCommand()));
        operatorController
            .povRight()
            .and(() -> shooter.coralNotDetectedByEitherSensor()
                || operatorController.leftBumper().getAsBoolean())
            .onTrue(elevator
                .setRequestedPositionCommand(elevator.LEVELALGAEPROC)
                .alongWith(shooter.deployAlgaeHookCommand()));
  
        operatorController
            .povDown()
            .and(() -> shooter.coralNotDetectedByEitherSensor()
                || operatorController.leftBumper().getAsBoolean())
            .onTrue(elevator.setRequestedPositionCommand(elevator.LEVELALGAE2));
        operatorController
            .povLeft()
            .and(() -> shooter.coralNotDetectedByEitherSensor()
                || operatorController.leftBumper().getAsBoolean())
            .onTrue(elevator.setRequestedPositionCommand(elevator.LEVELBARGE));
            
        // Shoots the coral out of the shooter
        operatorController.x().whileTrue(shooter.spitOutCoralCommand());
      }
  }

  private void initDrivetrain(Drivetrain.Config config) {
    drivetrain = Drivetrain.makeDrivetrain(config);

    if (drivetrain != null) {
      drive = new SwerveRequest.FieldCentric()
          .withDriveRequestType(
              DriveRequestType.Velocity); // Use open-loop control for drive motors

      reefLock = new FieldCentricFacingAngle()
          .withDeadband(Drivetrain.MAX_LINEAR_SPEED.times(0.1))
          .withDriveRequestType(DriveRequestType.Velocity);
      reefLock.HeadingController.setP(4);
      reefLock.HeadingController.setI(0);
      reefLock.HeadingController.setD(0);

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

      configureDriverBindings(config);
      registerDriverNamedCommands(config);
    }
  }
  public Command getAutonomousCommand() {
    return null;
  }
    
}