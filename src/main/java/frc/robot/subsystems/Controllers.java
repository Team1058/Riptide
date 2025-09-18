package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controllers extends SubsystemBase {
  private CommandXboxController driverController;
  private CommandXboxController operaterController;

  public Controllers(
      CommandXboxController driverController, CommandXboxController operaterController) {
    this.driverController = driverController;
    this.operaterController = operaterController;
    setDefaultCommand(runOnce(() -> this.driverController.setRumble(RumbleType.kBothRumble, 0)));
  }

  public Command rumbleControllersCommand(boolean rumbleDriver, boolean rumbleOperator) {
    return new StartEndCommand(
            () -> {
              if (rumbleDriver) {
                driverController.setRumble(RumbleType.kBothRumble, 1);
              }
              if (rumbleOperator) {
                operaterController.setRumble(RumbleType.kBothRumble, 1);
              }
            },
            () -> {
              driverController.setRumble(RumbleType.kBothRumble, 0);
              operaterController.setRumble(RumbleType.kBothRumble, 0);
            })
        .withName("Controllers Rumble Command");
  }

  private boolean isOutsideDeadband(double value) {
    return Math.abs(value) > .1;
  }

  public double getDriverLeftY() {
    if (isOutsideDeadband(driverController.getLeftY())) {
      return driverController.getLeftY();
    }

    return 0;
  }

  public double getDriverRightY() {
    if (isOutsideDeadband(driverController.getRightY())) {
      return driverController.getRightY();
    }

    return 0;
  }

  public double getDriverLeftX() {
    if (isOutsideDeadband(driverController.getLeftX())) {
      return driverController.getLeftX();
    }

    return 0;
  }

  public double getDriverRightX() {
    if (isOutsideDeadband(driverController.getRightX())) {
      return driverController.getRightX();
    }

    return 0;
  }

  public double getOperatorLeftY() {
    if (isOutsideDeadband(operaterController.getLeftY())) {
      return operaterController.getLeftY();
    }

    return 0;
  }

  public double getOperatorRightY() {
    if (isOutsideDeadband(operaterController.getRightY())) {
      return operaterController.getRightY();
    }

    return 0;
  }
}
