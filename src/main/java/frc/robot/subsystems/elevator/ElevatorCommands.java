package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import pvc.runtime.SubsystemCommands;

public class ElevatorCommands extends SubsystemCommands {

  private Elevator elevator;

  public ElevatorCommands(Elevator elevator) {
    this.elevator = elevator;
  }

  public Command goToPosition(Elevator.Position position) {
    return new FunctionalCommand(
        () -> elevator.setPosition(position),
        () -> {},
        (interrupted) -> {
          if (interrupted) {
            elevator.holdPosition();
          }
        },
        () -> elevator.isAtPosition(position),
        this);
  }
}
