package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.TunableConstant;
import frc.robot.utils.TunablePID;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

// This is the coral cannon
public class Shooter extends SubsystemBase {
  private SparkLimitSwitch inLimitSwitch;
  private SparkLimitSwitch outLimitSwitch;

  private RelativeEncoder algaeEncoder;

  public static class Config {
    public int shooterMotorId;
    public boolean invertMotors;
    public int algaeMotorId;
    public int intakeFlapMotorID;
    public double flapForwardLimit;
    public double flapReversedLimit;
    public boolean hasAlgaeMotor;
    public double algaeKP;
    public double algaeKI;
    public double algaeKD;
    public double algaeKF;
  }

  public TunableConstant STOWED;
  // public double HOLD = 0.56;
  public TunableConstant HOLD;
  public TunableConstant DEPLOYEDREEF;
  public TunableConstant DEPLOYEDFLOOR;

  private SparkFlex shooterMotor;
  private SparkMax intakeFlapMotor;
  public SparkMax algaeMotor = null;
  public SparkClosedLoopController algaePositionController = null;
  private SparkMaxConfig algaeMotorConfig;
  private SparkFlexConfig motorConfig;
  private SparkMaxConfig intakeFlapMotorConfig;

  private Config config;


  private TunablePID algaePid;
  public Shooter(Config config) {
    this.config = config;
    shooterMotor = new SparkFlex(config.shooterMotorId, MotorType.kBrushless);
    intakeFlapMotor = new SparkMax(config.intakeFlapMotorID, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();
    algaeMotorConfig = new SparkMaxConfig();
    intakeFlapMotorConfig = new SparkMaxConfig();
    algaePid = new TunablePID("/AlgeaMech/AlgaePid", config.algaeKP, config.algaeKI, config.algaeKD, config.algaeKF);
    STOWED = new TunableConstant("/AlgaeMech/Stowed", -7);
    HOLD = new TunableConstant("/AlgaeMech/HOLD", -7);
    DEPLOYEDREEF = new TunableConstant("/AlgaeMech/DeployedReef", -13.5);
    DEPLOYEDFLOOR = new TunableConstant("/AlgaeMech/DeployedFloor", -22);


    if (config.hasAlgaeMotor) {
      algaeMotor = new SparkMax(config.algaeMotorId, MotorType.kBrushless);
      algaeEncoder = algaeMotor.getEncoder();

      algaeMotorConfig
          .smartCurrentLimit(20, 20)
          .idleMode(IdleMode.kBrake)
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(config.algaeKP, config.algaeKI, config.algaeKD, config.algaeKF);
      algaeMotorConfig.softLimit.forwardSoftLimit(DEPLOYEDREEF.getAndUpdate() + .5).forwardSoftLimitEnabled(false);
      algaeMotorConfig.softLimit.reverseSoftLimit(STOWED.getAndUpdate()).reverseSoftLimitEnabled(false);

      algaeMotor.configure(
          algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      algaePositionController = algaeMotor.getClosedLoopController();
    }

    motorConfig
        .smartCurrentLimit(90, 90)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(0.1)
        .limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(false);

    intakeFlapMotorConfig.smartCurrentLimit(90, 90)
        .idleMode(IdleMode.kBrake)
        .softLimit.forwardSoftLimit(config.flapForwardLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(config.flapReversedLimit)
        .reverseSoftLimitEnabled(true);

    intakeFlapMotor.configure(
      intakeFlapMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    algaeEncoder.setPosition(0);

    inLimitSwitch = shooterMotor.getForwardLimitSwitch();
    outLimitSwitch = shooterMotor.getReverseLimitSwitch();

  }


  public boolean coralNotDetectedByEitherSensor() {
    return !inLimitSwitch.isPressed() && !outLimitSwitch.isPressed();
  }

  public boolean coralDetectedByEitherSensor() {
    return inLimitSwitch.isPressed() || outLimitSwitch.isPressed();
  }

  public boolean coralDetectedByOutSensor() {
    return outLimitSwitch.isPressed();
  }

  public boolean coralDetectedByInSensor() {
    return inLimitSwitch.isPressed();
  }
//needs fixing
  public boolean algaeMechDeployed(){
    return algaeEncoder.getPosition() == DEPLOYEDREEF.getAndUpdate();
  }
  // and provide commands to set rumble state
  public Trigger bothSensorsDetectCoral() {
    return new Trigger(() -> inLimitSwitch.isPressed() && outLimitSwitch.isPressed());
  }

  public Trigger coralDetectedByInSensorOnly() {
    return new Trigger(() -> inLimitSwitch.isPressed() && !outLimitSwitch.isPressed());
  }

  public Command manualShootCommand(DoubleSupplier supplier) {
    return runEnd(
      () -> shooterMotor.set(supplier.getAsDouble()),
      () -> shooterMotor.disable())
      .withName("Manual Shoot Command");
  }

  public Command spitOutCoralCommand() {
    return new StartEndCommand(
      () -> shooterMotor.set(-0.3),
      () -> shooterMotor.disable(),
      this)
      .withName("Spit Out Coral Command");
  }

  
  public Command unIntakeCommand() {
    return new StartEndCommand(
      () -> shooterMotor.set(1),
      () -> shooterMotor.disable(),
      this)
      .withName("Reverse the intake");
  }

  public Command timedSpitCoralCommand(double time) {
    return Commands.race(spitOutCoralCommand(), Commands.waitSeconds(time));
  }

  public Command deployAlgaeHookCommand() {
    return runOnce(
      () -> algaePositionController.setReference(DEPLOYEDREEF.getAndUpdate(), ControlType.kPosition))
        .withName("Deploy Algae Hook Command");
  }

  public Command holdAlgaeHookCommand() {
    return runOnce(
      () -> algaePositionController.setReference(HOLD.getAndUpdate(), ControlType.kPosition))
        .withName("Hold Algae Hook Command");
  }

  public Command stowAlgaeHookCommand() {
    return runOnce(
      () -> algaePositionController.setReference(-7, ControlType.kPosition))
        .withName("Stow Algae Hook Command");
  }

  public boolean algaeMotorAtTarget(double target) {
    return Math.abs(algaeMotor.getEncoder().getPosition() - target) < 0.02;
  }

  public Command intakeAlgae() {
    return new StartEndCommand(
      () -> shooterMotor.set(.5),
      () -> shooterMotor.set(.2),
      this)
      .withName("Intake Algae");
  }

  public Command shootAlgae() {
    return new StartEndCommand(
            () -> shooterMotor.set(-1),
            () -> {
              shooterMotor.disable();
              algaePositionController.setReference(STOWED.getAndUpdate(), ControlType.kPosition);
            },
            this)
        .withName("Shoot Algae");
  }

  public Command runShooterInFastUntilInLimitTriggered() {
    return new FunctionalCommand(
      ()-> shooterMotor.set(-0.1),
      ()-> {},
      interupted -> shooterMotor.disable(),
      ()-> outLimitSwitch.isPressed(),
      this).withName("Runs shooter fast til' the in and out limit switch is triggered.");
  }


  public Command runShooterInSlowUntilOutLimitTriggered() {
    return new FunctionalCommand(
      ()-> shooterMotor.set(-0.1),
      ()-> {},
      interrupted -> shooterMotor.disable(),
      ()-> outLimitSwitch.isPressed() && !inLimitSwitch.isPressed(),
      this).withName("Runs shooter in slow until the out limit switch is triggered.");
  }

  public Command runShooterOutSlowUntilOutLimitTriggeredThenStop() {
    return new FunctionalCommand(
      ()-> shooterMotor.set(0.15),
      ()-> {},
      interrupted -> shooterMotor.disable(),
      ()-> outLimitSwitch.isPressed(),
      this).withName("Runs the shooter in slow until out limit switch is triggered then stop the shooter.");
  }

  public Command intakeCoralCommand() {
    return runShooterInFastUntilInLimitTriggered()
      .andThen(runShooterOutSlowUntilOutLimitTriggeredThenStop())
      .withName("Coral intake command");
  }

  public Command openSesameCommand() {
    return pullPinCommand()
      .andThen(unPullPinCommand())
      .withName("Open intake flaps");
  }

  private Command pullPinCommand() {
    return new StartEndCommand(
      ()-> intakeFlapMotor.set(-0.5), 
      ()->intakeFlapMotor.disable(), 
      this)
      .until(()-> intakeFlapMotor.getEncoder().getPosition() <= config.flapReversedLimit + 0.1);
    }

  private Command unPullPinCommand() {
    return new StartEndCommand(
      ()-> intakeFlapMotor.set(0.5), 
      ()->intakeFlapMotor.disable(), 
      this)
      .until(()-> intakeFlapMotor.getEncoder().getPosition() >= config.flapForwardLimit - 0.1);
  }

  public void setAllMotorsBrake() {
    if (config.hasAlgaeMotor) {
      algaeMotorConfig.idleMode(IdleMode.kBrake);
      algaeMotor.configure(
          algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    motorConfig.idleMode(IdleMode.kBrake);
    shooterMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setAllMotorsCoast() {
    if (config.hasAlgaeMotor) {
      algaeMotorConfig.idleMode(IdleMode.kCoast);
      algaeMotor.configure(
          algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    motorConfig.idleMode(IdleMode.kCoast);
    shooterMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter In Limit Switch", inLimitSwitch.isPressed());
    Logger.recordOutput("Shooter Out Limit Switch", outLimitSwitch.isPressed());
    Logger.recordOutput("Algae position", algaeEncoder.getPosition());
  }

    private void updateAlgaePid(double kP, double kI, double kD){
    algaeMotorConfig
    .closedLoop
    .pid(kP, kI, kD);
    algaeMotor.configure(
      algaeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void testPeriodic() {
    
      algaePid.updatePID((algaekP, algaekI, algaekD) -> updateAlgaePid(algaekP, algaekI, algaekD));

  }

  public void testInit() {
    STOWED.setTuningMode(true);
    HOLD.setTuningMode(true);
    DEPLOYEDFLOOR.setTuningMode(true);
    DEPLOYEDREEF.setTuningMode(true);
    algaePid.setTuningMode(true);
  }
}
