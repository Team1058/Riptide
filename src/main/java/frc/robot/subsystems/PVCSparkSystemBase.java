package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public abstract class PVCSparkSystemBase extends SubsystemBase {

  @AutoLog
  public static class SubsystemInputs {
    public double appliedOutput = 0.0;
    public double outputCurrent = 0.0;
    public boolean forwardLimitSwitch = false;
    public boolean reverseLimitSwitch = false;
    public double absoluteEncoderPosition = 0.0;
    public double absoluteEncoderVelocity = 0.0;
    public double relativeEncoderPosition = 0.0;
    public double relativeEncoderVelocity = 0.0;
    public double[] setpoint = new double[] {0.0, 0.0, 0.0, 0.0};
    public boolean[] atSetpoint = new boolean[] {false, false, false, false};
    public double temperature = 0.0;
    public boolean faulted = false;
    public boolean connected = true;
    public boolean followerFaulted = false;
    public boolean followerConnected = true;
  }

  protected SubsystemInputs subsystemInputs;
  protected SparkBase primaryMotor;

  public PVCSparkSystemBase(String name) {
    super(name);
  }

  public abstract void setAllMotorsBrake();

  public abstract void setAllMotorsCoast();

  protected void updateInputs() {
    subsystemInputs.appliedOutput = primaryMotor.getAppliedOutput();
    subsystemInputs.outputCurrent = primaryMotor.getOutputCurrent();
    subsystemInputs.forwardLimitSwitch = primaryMotor.getForwardLimitSwitch().isPressed();
    subsystemInputs.reverseLimitSwitch = primaryMotor.getReverseLimitSwitch().isPressed();
    subsystemInputs.absoluteEncoderPosition = primaryMotor.getAbsoluteEncoder().getPosition();
    subsystemInputs.absoluteEncoderVelocity = primaryMotor.getAbsoluteEncoder().getVelocity();
    subsystemInputs.relativeEncoderPosition = primaryMotor.getEncoder().getPosition();
    subsystemInputs.relativeEncoderVelocity = primaryMotor.getEncoder().getVelocity();
    subsystemInputs.temperature = primaryMotor.getMotorTemperature();
    subsystemInputs.faulted = subsystemInputs.faulted || primaryMotor.hasStickyFault();
    subsystemInputs.connected = primaryMotor.clearFaults() != REVLibError.kOk;
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs(getName(), (SubsystemInputsAutoLogged) subsystemInputs);
  }
}
