package pvc.runtime;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Runtime {

  public <InputsT extends LoggableInputs, OutputsT extends LoggableInputs> void installSubsystem(
      Subsystem<InputsT, OutputsT> subsystem,
      SubsystemHardware<InputsT, OutputsT> hardware,
      InputsT inputs,
      OutputsT outputs) {
    var driver = new SubSystemDriver(subsystem) {
      @Override
      public void readAndProcessInputs(RobotState robotState) {
        hardware.readInputs(inputs);
        Logger.processInputs("", inputs);
        subsystem.processInputs(inputs, robotState);
      }

      @Override
      public void generateAndWriteOutputs(RobotState robotState) {
        subsystem.generateOutputs(outputs, robotState);
        hardware.writeOutputs(outputs);
      }
    };
  }

  void runCycle() {}
}
