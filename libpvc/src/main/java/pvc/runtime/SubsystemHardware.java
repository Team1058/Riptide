package pvc.runtime;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SubsystemHardware<
    AutoLogInputsT extends LoggableInputs, AutoLogOutputsT extends LoggableInputs> {

  public void readInputs(AutoLogInputsT inputs);

  public void writeOutputs(AutoLogOutputsT outputs);
}
