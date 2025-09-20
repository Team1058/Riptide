package pvc.runtime;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface Subsystem<InputsT extends LoggableInputs, OutputsT extends LoggableInputs> {

  public abstract void processInputs(InputsT inputs, RobotState robotState);

  public abstract void generateOutputs(OutputsT outputs, RobotState robotState);

  /** Called before subsystem inputs are read and processed. */
  default void preInput(RobotState robotState) {}

  /** Called before subsystem inputs are read and processed. */
  default void postInput(RobotState robotState) {}

  /** Called when the robot state transitions from disabled to enabled.
   *
   * This is called after inputs are processed but before the commmand scheduler is run. Inputs have already been read
   * and can be used to place the subsystem in an appropriate state if no commands end up being scheduled this cycle.
   */
  default void enable(RobotMode enterningMode) {}

  /** Called when the robot state transitions from enabled to disabled.
   *
   * This is called after the command scheduler runs but before outputs are written. All commands that don't run when
   * disabled will have already been canceled. The subsystem can be placed in an appropriate state for disabled mode
   * without commands overriding it.
   */
  default void disable(RobotMode exitingMode) {}

  /** Called after before outputs are generated and written. */
  default void preOutput(RobotState robotState) {}

  /** Called after outputs are generated and written. */
  default void postOutput(RobotState robotState) {}
}
