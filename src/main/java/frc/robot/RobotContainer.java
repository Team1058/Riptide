package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Controllers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  Climber climber;
  Shooter shooter;
  Elevator elevator;
  RobotConfig robotConfig;
  int driverPort = 0;
  int operatorPort = 1;
  Controllers controllers;

  CommandXboxController operatorController;
  CommandXboxController driveController;
  

  public RobotContainer() {
    driveController = new CommandXboxController(robotConfig.driverControllerPort);
    operatorController = new CommandXboxController(robotConfig.operatorControllerPort);

    String serialNumber = System.getenv("serialnum");
    RoboRio roboRio = RoboRio.lookupBySerialNumber(serialNumber);
    robotConfig = RobotConfig.lookupConfig(roboRio);
    climber = new Climber(robotConfig.climberConfig);
    elevator = new Elevator(robotConfig.elevatorConfig);
    shooter = new Shooter(robotConfig.shooterConfig);
    configureOperatorBindings();
  }


  private void configureOperatorBindings() {
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
        operatorController.a()
        .whileTrue(elevator.setRequestedPositionCommand(elevator.LEVELHP)
        .andThen(elevator.goToRequestedPositionCommand())
        .andThen(shooter.intakeCoralCommand()));
        
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

        operatorController
            .back()
            .and(operatorController.start())
            .onTrue(new RunCommand(() -> swapControllers(), controllers)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
            
        // Shoots the coral out of the shooter
        operatorController.x()
        .whileTrue(shooter.spitOutCoralCommand());
      }
  }

  public Command getAutonomousCommand() {
    return null;
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
    }
}
