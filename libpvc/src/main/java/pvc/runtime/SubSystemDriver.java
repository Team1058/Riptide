package pvc.runtime;

public abstract class SubSystemDriver {

  private Subsystem<?, ?> subsystem;

  public SubSystemDriver(Subsystem<?, ?> subsystem) {
    this.subsystem = subsystem;
  }

  /** Called before subsystem inputs are read and processed. */
  public void preInput(RobotState robotState) {
    subsystem.preInput(robotState);
  }

  public abstract void readAndProcessInputs(RobotState robotState);

  /** Called before subsystem inputs are read and processed. */
  public void postInput(RobotState robotState) {
    subsystem.postInput(robotState);
  }

  /** Called when the robot state transitions from disabled to enabled.
   *
   * This is called after inputs are processed but before the commmand scheduler is run. Inputs have already been read
   * and can be used to place the subsystem in an appropriate state if no commands end up being scheduled this cycle.
   */
  public void enable(RobotMode enterningMode) {
    subsystem.enable(enterningMode);
  }

  /** Called when the robot state transitions from enabled to disabled.
   *
   * This is called after the command scheduler runs but before outputs are written. All commands that don't run when
   * disabled will have already been canceled. The subsystem can be placed in an appropriate state for disabled mode
   * without commands overriding it.
   */
  public void disable(RobotMode exitingMode) {
    subsystem.disable(exitingMode);
  }

  /** Called after before outputs are generated and written. */
  public void preOutput(RobotState robotState) {
    subsystem.preOutput(robotState);
  }

  public abstract void generateAndWriteOutputs(RobotState robotState);

  /** Called after outputs are generated and written. */
  public void postOutput(RobotState robotState) {
    subsystem.postOutput(robotState);
  }
}
