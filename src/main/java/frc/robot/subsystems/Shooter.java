package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

// This is the coral cannon
public class Shooter extends SubsystemBase {
  private SparkLimitSwitch inLimitSwitch;
  private SparkLimitSwitch outLimitSwitch;

  private SparkAbsoluteEncoder algaeEncoder;

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

  public double STOWED = 0.15;
  // public double HOLD = 0.56;
  public double HOLD = 0.45;
  public double DEPLOYED = 0.80;

  private SparkFlex shooterMotor;
  private SparkMax intakeFlapMotor;
  public SparkMax algaeMotor = null;
  private SparkClosedLoopController algaePositionController = null;
  private SparkMaxConfig algaeMotorConfig;
  private SparkFlexConfig motorConfig;
  private SparkMaxConfig intakeFlapMotorConfig;

  private Config config;

  private GenericEntry pEntry;
  private GenericEntry iEntry;
  private GenericEntry dEntry;
  private GenericEntry fEntry;

  public Shooter(Config config) {
    this.config = config;
    shooterMotor = new SparkFlex(config.shooterMotorId, MotorType.kBrushless);
    intakeFlapMotor = new SparkMax(config.intakeFlapMotorID, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();
    algaeMotorConfig = new SparkMaxConfig();
    intakeFlapMotorConfig = new SparkMaxConfig();

    if (config.hasAlgaeMotor) {
      algaeMotor = new SparkMax(config.algaeMotorId, MotorType.kBrushless);
      algaeEncoder = algaeMotor.getAbsoluteEncoder();

      algaeMotorConfig
          .smartCurrentLimit(20, 20)
          .idleMode(IdleMode.kBrake)
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pidf(config.algaeKP, config.algaeKI, config.algaeKD, config.algaeKF);
      algaeMotorConfig.softLimit.forwardSoftLimit(DEPLOYED + .5).forwardSoftLimitEnabled(false);
      algaeMotorConfig.softLimit.reverseSoftLimit(STOWED).reverseSoftLimitEnabled(false);

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
        .softLimit.forwardSoftLimit(config.flapForwardLimit).forwardSoftLimitEnabled(true)
        .reverseSoftLimit(config.flapReversedLimit).reverseSoftLimitEnabled(true);

    intakeFlapMotor.configure(
      intakeFlapMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterMotor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    inLimitSwitch = shooterMotor.getForwardLimitSwitch();
    outLimitSwitch = shooterMotor.getReverseLimitSwitch();

    initializeShuffleboardEntries();
  }

  private void initializeShuffleboardEntries() {
    pEntry = Shuffleboard.getTab("PID")
        .add("Algae P", config.algaeKP)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    iEntry = Shuffleboard.getTab("PID")
        .add("Algae I", config.algaeKI)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    dEntry = Shuffleboard.getTab("PID")
        .add("Algae D", config.algaeKD)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    fEntry = Shuffleboard.getTab("PID")
        .add("Algae F", config.algaeKF)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
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
    return algaeEncoder.getPosition() == DEPLOYED;
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
      () -> shooterMotor.set(-0.5),
      () -> shooterMotor.disable(),
      this)
      .withName("Spit Out Coral Command");
  }

  public Command deployAlgaeHookCommand() {
    return runOnce(
      () -> algaePositionController.setReference(DEPLOYED, ControlType.kPosition))
        .withName("Deploy Algae Hook Command");
  }

  public Command holdAlgaeHookCommand() {
    return runOnce(
      () -> algaePositionController.setReference(HOLD, ControlType.kPosition))
        .withName("Hold Algae Hook Command");
  }

  public Command stowAlgaeHookCommand() {
    return runOnce(
      () -> algaePositionController.setReference(STOWED, ControlType.kPosition))
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
              algaePositionController.setReference(STOWED, ControlType.kPosition);
            },
            this)
        .withName("Shoot Algae");
  }

  public Command runShooterInFastUntilInLimitTriggered() {
    return new FunctionalCommand(
      ()-> shooterMotor.set(-0.25),
      ()-> {},
      interupted -> shooterMotor.disable(),
      ()-> inLimitSwitch.isPressed(),
      this).withName("Runs shooter fast til' the in and out limit switch is triggered.");
  }


  public Command runShooterInSlowUntilOutLimitTriggered() {
    return new FunctionalCommand(
      ()-> shooterMotor.set(-0.075),
      ()-> {},
      interrupted -> shooterMotor.disable(),
      ()-> outLimitSwitch.isPressed(),
      this).withName("Runs shooter in slow until the out limit switch is triggered.");
  }

  public Command runShooterOutSlowUntilInAndOutLimitTriggeredThenStop() {
    return new FunctionalCommand(
      ()-> shooterMotor.set(0.15),
      ()-> {},
      interrupted -> shooterMotor.disable(),
      ()-> inLimitSwitch.isPressed() && outLimitSwitch.isPressed(),
      this).withName("Runs the shooter in slow until the in and out limit switch is triggered then stop the shooter.");
  }

  public Command intakeCoralCommand() {
    return runShooterInFastUntilInLimitTriggered()
      .andThen(runShooterInSlowUntilOutLimitTriggered())
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

  public void testPeriodic() {
    double pFromShuffleBoard = pEntry.getDouble(config.algaeKP);
    double iFromShuffleBoard = iEntry.getDouble(config.algaeKI);
    double dFromShuffleBoard = dEntry.getDouble(config.algaeKD);
    double fFromShuffleBoard = fEntry.getDouble(config.algaeKF);

    double currentP = algaeMotor.configAccessor.closedLoop.getP();
    double currentI = algaeMotor.configAccessor.closedLoop.getI();
    double currentD = algaeMotor.configAccessor.closedLoop.getD();
    double currentF = algaeMotor.configAccessor.closedLoop.getFF();

    if (valuesActuallyDifferent(pFromShuffleBoard, currentP)
        || valuesActuallyDifferent(iFromShuffleBoard, currentI)
        || valuesActuallyDifferent(dFromShuffleBoard, currentD)
        || valuesActuallyDifferent(fFromShuffleBoard, currentF)) {

      algaeMotorConfig.closedLoop.pidf(
          pFromShuffleBoard, iFromShuffleBoard, dFromShuffleBoard, fFromShuffleBoard);

      algaeMotor.configure(
          algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  private boolean valuesActuallyDifferent(double val1, double val2) {
    return Math.abs(val1 - val2) > .000001;
  }
}
