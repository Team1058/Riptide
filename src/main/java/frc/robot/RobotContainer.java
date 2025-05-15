package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  Climber climber;
  Shooter shooter;
  Elevator elevator;
  Leds leds;

  RobotConfig robotConfig;


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
    configureBindings();
  }


  private void configureBindings() {
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

  public void ledSetPatternsLogic(){

    leds.applyPatternsToStrips();

    if (shooter.getCurrentCommand().getName().equals("coral intake command")
    || shooter.getCurrentCommand().getName().equals("Manual Shoot Command")){
        leds.leftPattern = leds.greenBase;
    }
    else{
        leds.leftPattern = leds.redOrangeBlinkWithRsl;
    }
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
