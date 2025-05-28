package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  public static class Config {
    public int climberLeaderMotorId;
    public int climberFollowerMotorId;
    public boolean invertLeaderMotor;
    public double climberClimbed;
    public double climberDeployed;
  }

  public boolean isClimbing = false;
  private SparkMax leaderMotor;
  private SparkMax followerMotor;
  private SparkMaxConfig leaderConfig;
  private SparkMaxConfig followerConfig;
  private SparkAbsoluteEncoder climberEncoder;
  private Config config;

  public Climber(Config config) {
    this.config = config;
    leaderMotor = new SparkMax(config.climberLeaderMotorId, MotorType.kBrushless);
    leaderConfig = new SparkMaxConfig();
    climberEncoder = leaderMotor.getAbsoluteEncoder();
    // TODO: Can we make these be based on the absolute encoder values?
    leaderConfig
        .smartCurrentLimit(80, 80)
        .idleMode(IdleMode.kBrake)
        .inverted(config.invertLeaderMotor)
        .openLoopRampRate(0.05)
        .softLimit
        .reverseSoftLimit(config.climberClimbed)
        .reverseSoftLimitEnabled(false)
        .forwardSoftLimit(config.climberDeployed)
        .forwardSoftLimitEnabled(false);
    leaderConfig.limitSwitch.forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false);

    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerMotor = new SparkMax(config.climberFollowerMotorId, MotorType.kBrushless);
    followerConfig = new SparkMaxConfig();
    followerConfig
    .follow(config.climberLeaderMotorId, true);

    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command manualClimbCommand(DoubleSupplier supplier) {
    return new FunctionalCommand(
            () -> {},
            () -> leaderMotor.set(Math.abs(supplier.getAsDouble())),
            interrupted -> leaderMotor.disable(),
            () -> climberEncoder.getPosition() >= config.climberClimbed,
            this)
        .withName("Manual Climb Command");
  }

  public Command deployClimberCommand() {
    return new FunctionalCommand(
            () -> {
              isClimbing = true;
              leaderMotor.set(1);
            },
            () -> {},
            interrupted -> leaderMotor.disable(),
            () -> climberEncoder.getPosition() >= config.climberDeployed,
            this)
        .withName("Deploy Climber Command");
  }

  public Trigger readyToClimbTrigger() {
    // TODO: Should this be a final class level?
    return new Trigger(() -> climberEncoder.getPosition() >= config.climberDeployed);
  }

  public void setAllMotorsBrake() {
    leaderConfig.idleMode(IdleMode.kBrake);
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setAllMotorsCoast() {
    leaderConfig.idleMode(IdleMode.kCoast);
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climber encoder ", climberEncoder.getPosition());
    Logger.recordOutput("Climber current", leaderMotor.getOutputCurrent());
  }

  public Trigger getIsClimbingTrigger() {
    return new Trigger(() -> isClimbing);
  }
}