// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class TunablePID {

  // Instance name for tagging log values
  String name;

  // Tunable numbers
  private TunableConstant kP, kI, kD, kF;

  public TunablePID(String name, double kP, double kI, double kD, double kF) {
    this.name = name;

    // Tunable numbers for PID and motion gain constants
    this.kP = new TunableConstant(name + "/kP", kP);
    this.kI = new TunableConstant(name + "/kI", kI);
    this.kD = new TunableConstant(name + "/kD", kD);
    this.kF = new TunableConstant(name + "/kF", kF);
  }

  public double getKF() {
    return kF.get();
  }

  public void updatePID(PidConsumer callback) {
    // If changed, update controller constants from Tuneable Numbers
    TunableConstant.ifChanged(
        values -> callback.accept(values[0], values[1], values[2]), kP, kI, kD);
  }

  public void setTuningMode(boolean enabled) {
    kP.setTuningMode(enabled);
    kI.setTuningMode(enabled);
    kD.setTuningMode(enabled);
    kF.setTuningMode(enabled);
  }
}
