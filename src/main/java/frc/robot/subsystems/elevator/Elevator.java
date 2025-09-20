package frc.robot.subsystems.elevator;

import pvc.runtime.RobotState;
import pvc.runtime.RobotStateEntry;
import pvc.runtime.Subsystem;

public class Elevator implements Subsystem<ElevatorInputsAutoLogged, ElevatorOutputsAutoLogged> {

  public static class Position {
    private Position(String name, double default_position) {
      this.name = name;
      this.position = default_position;
    }

    String name;
    double position;
  }

  public Elevator(ElevatorConfig config) {}

  public static final RobotStateEntry<Double> elevatorPosition =
      RobotState.make_entry("ElevatorPosition", 0.0);

  final Position CORAL_L1 = new Position("", 0);
  final Position CORAL_L2 = new Position("", 0);

  enum Positions {
    CORAL_L1,
    CORAL_L2,
    CORAL_L3,
    CORAL_l4,
    ALGAE_FLOOR,
    ALGAE_LOLLIPOP,
    ALGAE_REEF_MID,
    ALGAE_REEF_HIGH,
    ALGAE_BARGE,
  }

  private enum State {
    hold,
    move,
    moving,
  }

  private State current_state;

  private double current_position;
  private Position target_position;
  private double command_position;

  public boolean isAtPosition(Position position) {
    return current_state == State.hold;
  }

  public boolean isAbove(Position position) {
    return false;
  }

  public void setPosition(Position position) {
    if (!isAtPosition(position)) {
      target_position = position;
      current_state = State.move;
    } else {
      current_state = State.hold;
    }
  }

  public void holdPosition() {}

  public void setPercentOutput(double percent_output) {}

  @Override
  public void processInputs(ElevatorInputsAutoLogged inputs, RobotState robotState) {}

  @Override
  public void generateOutputs(ElevatorOutputsAutoLogged outputs, RobotState robotState) {
    switch (current_state) {
      case hold:
        break;
      case move:
        command_position = target_position.position; // make command position drive motor
        current_state = State.moving;
        break;
      case moving:
        if (isAtPosition(target_position)) {
          current_state = State.hold;
        }
    }
  }
}
