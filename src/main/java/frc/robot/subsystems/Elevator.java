package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  public static class Config {
    public int leaderMotorId;
    public int followerMotorId;
    public int followerMotor2Id;
    public boolean hasFollowerMotor2;
    public boolean invertLeaderMotor;

    public double kP_Up;
    public double kI_Up;
    public double kD_Up;
    public double kF_Up;
    public double maxVelocity_Up;
    public double maxAcceleration_Up;
    public double allowedError_Up;

    public double kP_Down;
    public double kI_Down;
    public double kD_Down;
    public double kF_Down;
    public double maxVelocity_Down;
    public double maxAcceleration_Down;
    public double allowedError_Down;

    public double upperLimit;
    public double lowerLimit;
  }

  public double LEVEL1 = 11.5 + 4.5;
  public double LEVEL2 = 22.5 + 4.5;
  public double LEVEL3 = 41.65 + 4.5;
  public double LEVEL4 = 77;

  public double LEVEL1NOSERVO = 11.5 + 3.5;
  public double LEVEL2NOSERVO = 25.5;
  public double LEVEL3NOSERVO = 44.65;
  public double LEVEL4NOSERVO = 75.5;

  public double LEVELALGAEPROC = 1;
  public double LEVELALGAELOLLIPOP = 1;
  public double LEVELALGAE1 = 20.25;
  public double LEVELALGAE2 = 39.25;
  public double LEVELBARGE = 81.3;
  public double LEVELHP = 1.0;

  private final SparkFlex leaderMotor;
  private final SparkFlex followerMotor;
  private final RelativeEncoder elevatorEncoder;
  private SparkFlex followerMotor2 = null;
  private SparkLimitSwitch bottomLimitSwitch;
  private SparkLimitSwitch topLimitSwitch;
  private SparkFlexConfig leaderConfig;
  private SparkFlexConfig followerConfig;
  private SparkFlexConfig follower2Config;
  private SparkClosedLoopController sparkPIDController;
  protected double requestedPosition;

  private GenericEntry pUpEntry;
  private GenericEntry iUpEntry;
  private GenericEntry dUpEntry;
  private GenericEntry fUpEntry;
  private GenericEntry maxUpVelocity;
  private GenericEntry maxUpAcceleration;
  private GenericEntry allowedErrorUp;

  private GenericEntry pDownEntry;
  private GenericEntry iDownEntry;
  private GenericEntry dDownEntry;
  private GenericEntry fDownEntry;
  private GenericEntry maxDownVelocity;
  private GenericEntry maxDownAcceleration;
  private GenericEntry allowedErrorDown;

  private int stallLimit = 90;
  private int freeLimit = 90;
  private double rampRate = .02;
  private Config config;
  protected Double positionToHoldWhenNotSafeToMoveElevator;

  private boolean zeroed = false;

  private final SysIdRoutine sysIdRoutine;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutAngle m_rotations = Rotations.mutable(0);
  private final MutAngularVelocity m_velocity = RPM.mutable(0);

  private static final ClosedLoopSlot downSlot = ClosedLoopSlot.kSlot0;
  private static final ClosedLoopSlot upSlot = ClosedLoopSlot.kSlot1;

  public Elevator(Config config) {

    this.config = config;
    leaderConfig = new SparkFlexConfig();
    followerConfig = new SparkFlexConfig();
    follower2Config = new SparkFlexConfig();
    leaderMotor = new SparkFlex(config.leaderMotorId, MotorType.kBrushless);
    followerMotor = new SparkFlex(config.followerMotorId, MotorType.kBrushless);
    leaderConfig
        .smartCurrentLimit(stallLimit, freeLimit)
        .inverted(config.invertLeaderMotor)
        .idleMode(IdleMode.kCoast)
        .closedLoopRampRate(rampRate);
    leaderConfig
        .softLimit
        .forwardSoftLimit(config.upperLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(config.lowerLimit)
        .reverseSoftLimitEnabled(true);
    leaderConfig
        .limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(false);

    leaderConfig
        .closedLoop
        .pid(config.kP_Up, config.kI_Up, config.kD_Up, upSlot)
        .pid(config.kP_Down, config.kI_Down, config.kD_Down, downSlot)
        .maxMotion
        .maxAcceleration(config.maxAcceleration_Up, upSlot)
        .maxVelocity(config.maxVelocity_Up, upSlot)
        .allowedClosedLoopError(config.allowedError_Up, upSlot)
        .maxAcceleration(config.maxAcceleration_Down, downSlot)
        .maxVelocity(config.maxVelocity_Down, downSlot)
        .allowedClosedLoopError(config.allowedError_Down, downSlot);

    followerConfig
        .follow(leaderMotor, true)
        .smartCurrentLimit(stallLimit, freeLimit)
        .idleMode(IdleMode.kCoast)
        .closedLoopRampRate(rampRate);

    if (config.hasFollowerMotor2 == true) {
      followerMotor2 = new SparkFlex(config.followerMotor2Id, MotorType.kBrushless);
      follower2Config
          .follow(leaderMotor, true)
          .smartCurrentLimit(stallLimit, freeLimit)
          .idleMode(IdleMode.kCoast)
          .closedLoopRampRate(rampRate);
      followerMotor2.configure(
          followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    System.out.println("p value " + leaderMotor.configAccessor.closedLoop.getP());

    requestedPosition = 0;
    bottomLimitSwitch = leaderMotor.getReverseLimitSwitch();
    sparkPIDController = leaderMotor.getClosedLoopController();
    topLimitSwitch = leaderMotor.getForwardLimitSwitch();
    elevatorEncoder = leaderMotor.getEncoder();

    positionToHoldWhenNotSafeToMoveElevator = null;

    if (bottomLimitSwitchPressed()) {
      zeroed = true;
    }

    initializeShuffleboardEntries();

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(7), Seconds.of(10)),
        new SysIdRoutine.Mechanism(
            leaderMotor::setVoltage,
            log -> {
              log.motor("elevator")
                  .voltage(m_appliedVoltage.mut_replace(
                      leaderMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(
                      m_rotations.mut_replace(elevatorEncoder.getPosition(), Rotations))
                  .angularVelocity(
                      m_velocity.mut_replace(elevatorEncoder.getVelocity(), RotationsPerSecond));
            },
            this));
  }

  private void initializeShuffleboardEntries() {
    pUpEntry = Shuffleboard.getTab("PID")
        .add("Elevator P Up", config.kP_Up)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    iUpEntry = Shuffleboard.getTab("PID")
        .add("Elevator I Up", config.kI_Up)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    dUpEntry = Shuffleboard.getTab("PID")
        .add("Elevator D Up", config.kD_Up)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    fUpEntry = Shuffleboard.getTab("PID")
        .add("Elevator F Up", config.kF_Up)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    maxUpVelocity = Shuffleboard.getTab("PID")
        .add("Elevator MaxVelocity Up", config.maxVelocity_Up)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    maxUpAcceleration = Shuffleboard.getTab("PID")
        .add("Elevator MaxAcceleartion Up", config.maxAcceleration_Up)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    allowedErrorUp = Shuffleboard.getTab("PID")
        .add("Allowed Error Up", config.allowedError_Up)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    pDownEntry = Shuffleboard.getTab("PID")
        .add("Elevator P Down", config.kP_Down)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    iDownEntry = Shuffleboard.getTab("PID")
        .add("Elevator I Down", config.kI_Down)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    dDownEntry = Shuffleboard.getTab("PID")
        .add("Elevator D Down", config.kD_Down)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    fDownEntry = Shuffleboard.getTab("PID")
        .add("Elevator F Down", config.kF_Down)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    maxDownVelocity = Shuffleboard.getTab("PID")
        .add("Elevator MaxVelocity Down", config.maxVelocity_Down)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    maxDownAcceleration = Shuffleboard.getTab("PID")
        .add("Elevator MaxAcceleartion Down", config.maxAcceleration_Down)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    allowedErrorDown = Shuffleboard.getTab("PID")
        .add("Allowed Error Down", config.allowedError_Down)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
  }

  public void setRequestedPosition(double requested) {
    this.requestedPosition = MathUtil.clamp(requested, config.lowerLimit, config.upperLimit);
  }

  public void setRequestedPositionToCurrentPosition() {
    this.setRequestedPosition(this.getCurrentPosition());
  }

  public Command runSysIdRoutine() {
    return (sysIdRoutine
            .quasistatic(Direction.kForward)
            .until(() -> elevatorEncoder.getPosition() > 70)
            .andThen(sysIdRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> elevatorEncoder.getPosition() < 2))
            .andThen(sysIdRoutine
                .dynamic(Direction.kForward)
                .until(() -> elevatorEncoder.getPosition() > 70))
            .andThen(sysIdRoutine
                .dynamic(Direction.kReverse)
                .until(() -> elevatorEncoder.getPosition() < 2))
            .andThen(Commands.print("DONE")))
        .withName(" Run Sys Id Rountine");
  }

  public Command manualDriveCommand(DoubleSupplier speedSupplier) {
    return run(() -> {
          double positionDelta = (speedSupplier.getAsDouble() * -1) * 2;
          double requested = this.getCurrentPosition() + positionDelta;
          this.setRequestedPosition(requested);

          if (requested > getCurrentPosition()) {
            sparkPIDController.setReference(requested, ControlType.kPosition, upSlot, config.kF_Up);
          } else {
            sparkPIDController.setReference(
                requested, ControlType.kPosition, downSlot, config.kD_Down);
          }
        })
        .withName("manualDrive")
        .withName("Manul Drive Command");
  }

  public Command resetElevatorCommand() {
    return new FunctionalCommand(
            () -> {
              this.setAllMotorsBrake();
              leaderConfig.softLimit.reverseSoftLimitEnabled(false);
              leaderMotor.configure(
                  leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            () -> leaderMotor.set(-.25),
            interrupted -> {
              if (!interrupted) {
                leaderMotor.set(0);
                elevatorEncoder.setPosition(0);
                zeroed = true;
                requestedPosition = 0;
                setRequestedPosition(LEVEL2);
              }
              leaderConfig.softLimit.reverseSoftLimitEnabled(true);
              leaderMotor.configure(
                  leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            () -> bottomLimitSwitch.isPressed(),
            this)
        .withName("Reset Elevator Command");
  }

  public boolean isAtPostion(double position) {
    return Math.abs(getCurrentPosition() - position) < config.allowedError_Up;
  }
  ;

  public double getCurrentPosition() {
    return elevatorEncoder.getPosition();
  }

  public Command goToRequestedPositionCommand() {

    return runOnce(() -> {

          double requested;

          if (!zeroed) {
            requested = getCurrentPosition();
          } else {
            requested = getRequestedPosition();
          }

          // if (requested- getCurrentPosition() < 0.1)
          if (requested > getCurrentPosition()) {
            sparkPIDController.setReference(requested, ControlType.kPosition, upSlot, config.kF_Up);
          } else {
            sparkPIDController.setReference(
                requested, ControlType.kPosition, downSlot, config.kF_Down);
          }
        })
        .withName("goToRequestedPosition");
  }

  public boolean currentPositionAtTarget(double target) {
    return Math.abs(this.getCurrentPosition() - target) < 0.2;
  }

  public boolean currentPositionAtTargetHP(double target) {
    return Math.abs(this.getCurrentPosition() - target) < 0.35;
  }

  public boolean bottomLimitSwitchPressed() {
    return bottomLimitSwitch.isPressed();
  }

  public boolean topLimitSwitchPressed() {
    return topLimitSwitch.isPressed();
  }

  public double getRequestedPosition() {
    return requestedPosition;
  }

  public void setAllMotorsBrake() {
    leaderConfig.idleMode(IdleMode.kBrake);
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerConfig.idleMode(IdleMode.kBrake);
    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    if (config.hasFollowerMotor2) {
      follower2Config.idleMode(IdleMode.kBrake);
      followerMotor2.configure(
          follower2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  public void setAllMotorsCoast() {
    leaderConfig.idleMode(IdleMode.kCoast);
    leaderMotor.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerConfig.idleMode(IdleMode.kCoast);
    followerMotor.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    if (config.hasFollowerMotor2) {
      follower2Config.idleMode(IdleMode.kCoast);
      followerMotor2.configure(
          follower2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Elevator Encoder Position", getCurrentPosition());
    Logger.recordOutput("Elevator Requested Position", getRequestedPosition());
    Logger.recordOutput("Bottom Limit Switch State", bottomLimitSwitchPressed());
    Logger.recordOutput("Top Limit Switch State", topLimitSwitchPressed());
    Logger.recordOutput("Leader Motor Output", leaderMotor.getAppliedOutput());
    Logger.recordOutput("Elevator Speed", leaderMotor.getEncoder().getVelocity());
    Logger.recordOutput("Elevator Current", leaderMotor.getOutputCurrent());
  }

  public Command setRequestedPositionCommand(double position) {
    return runOnce(() -> this.setRequestedPosition(position)).withName("setRequestedPosition");
  }

  public Command setRequestedToCurrentPositionCommand() {
    return runOnce(() -> holdPosition()).withName("setRequestedPositionFromSupplier");
  }

  public void holdPosition() {
    setRequestedPosition(getCurrentPosition());
  }

  public boolean isZeroed() {
    return zeroed;
  }

  public Trigger isZeroedTrigger() {
    return new Trigger(this::isZeroed);
  }

  public void testPeriodic() {
    double pUpFromShuffleBoard = pUpEntry.getDouble(config.kP_Up);
    double iUpFromShuffleBoard = iUpEntry.getDouble(config.kI_Up);
    double dUpFromShuffleBoard = dUpEntry.getDouble(config.kD_Up);
    double fUpFromShuffleBoard = fUpEntry.getDouble(config.kF_Up);
    double maxVelocityUpFromShuffleBoard = maxUpVelocity.getDouble(config.maxVelocity_Up);
    double maxAcclerationUpFromShuffleBoard =
        maxUpAcceleration.getDouble(config.maxAcceleration_Up);
    double allowedErrorUpFromShuffleBoard = allowedErrorUp.getDouble(config.allowedError_Up);

    double pDownFromShuffleBoard = pDownEntry.getDouble(config.kP_Down);
    double iDownFromShuffleBoard = iDownEntry.getDouble(config.kI_Down);
    double dDownFromShuffleBoard = dDownEntry.getDouble(config.kD_Down);
    double fDownFromShuffleBoard = fDownEntry.getDouble(config.kF_Down);
    double maxVelocityDownFromShuffleBoard = maxDownVelocity.getDouble(config.maxVelocity_Down);
    double maxAcclerationDownFromShuffleBoard =
        maxDownAcceleration.getDouble(config.maxAcceleration_Down);
    double allowedErrorDownFromShuffleBoard = allowedErrorDown.getDouble(config.allowedError_Down);

    double currentUpP = leaderMotor.configAccessor.closedLoop.getP(upSlot);
    double currentUpI = leaderMotor.configAccessor.closedLoop.getI(upSlot);
    double currentUpD = leaderMotor.configAccessor.closedLoop.getD(upSlot);
    double currentUpF = config.kF_Up; // leaderMotor.configAccessor.closedLoop.getFF(upSlot);
    double currentMaxVelocityUp =
        leaderMotor.configAccessor.closedLoop.maxMotion.getMaxVelocity(upSlot);
    double currentMaxAccelerationUp =
        leaderMotor.configAccessor.closedLoop.maxMotion.getMaxAcceleration(upSlot);
    double currentAllowedErrorUp =
        leaderMotor.configAccessor.closedLoop.maxMotion.getAllowedClosedLoopError(upSlot);

    double currentDownP = leaderMotor.configAccessor.closedLoop.getP(downSlot);
    double currentDownI = leaderMotor.configAccessor.closedLoop.getI(downSlot);
    double currentDownD = leaderMotor.configAccessor.closedLoop.getD(downSlot);
    double currentDownF = config.kF_Down; // leaderMotor.configAccessor.closedLoop.getFF(downSlot);
    double currentMaxVelocityDown =
        leaderMotor.configAccessor.closedLoop.maxMotion.getMaxVelocity(downSlot);
    double currentMaxAccelerationDown =
        leaderMotor.configAccessor.closedLoop.maxMotion.getMaxAcceleration(downSlot);
    double currentAllowedErrorDown =
        leaderMotor.configAccessor.closedLoop.maxMotion.getAllowedClosedLoopError(downSlot);

    if (valuesActuallyDifferent(pUpFromShuffleBoard, currentUpP)
        || valuesActuallyDifferent(iUpFromShuffleBoard, currentUpI)
        || valuesActuallyDifferent(dUpFromShuffleBoard, currentUpD)
        || valuesActuallyDifferent(fUpFromShuffleBoard, currentUpF)
        || valuesActuallyDifferent(maxAcclerationUpFromShuffleBoard, currentMaxAccelerationUp)
        || valuesActuallyDifferent(maxVelocityUpFromShuffleBoard, currentMaxVelocityUp)
        || valuesActuallyDifferent(allowedErrorUpFromShuffleBoard, currentAllowedErrorUp)
        || valuesActuallyDifferent(pDownFromShuffleBoard, currentDownP)
        || valuesActuallyDifferent(iDownFromShuffleBoard, currentDownI)
        || valuesActuallyDifferent(dDownFromShuffleBoard, currentDownD)
        || valuesActuallyDifferent(fDownFromShuffleBoard, currentDownF)
        || valuesActuallyDifferent(maxAcclerationDownFromShuffleBoard, currentMaxAccelerationDown)
        || valuesActuallyDifferent(maxVelocityDownFromShuffleBoard, currentMaxVelocityDown)
        || valuesActuallyDifferent(allowedErrorDownFromShuffleBoard, currentAllowedErrorDown)) {

      leaderConfig
          .closedLoop
          .pid(pUpFromShuffleBoard, iUpFromShuffleBoard, dUpFromShuffleBoard, upSlot)
          .pid(pDownFromShuffleBoard, iDownFromShuffleBoard, dDownFromShuffleBoard, downSlot)
          .maxMotion
          .maxAcceleration(maxAcclerationUpFromShuffleBoard, upSlot)
          .maxVelocity(maxVelocityUpFromShuffleBoard, upSlot)
          .allowedClosedLoopError(allowedErrorUpFromShuffleBoard, upSlot)
          .maxAcceleration(maxAcclerationDownFromShuffleBoard, downSlot)
          .maxVelocity(maxVelocityDownFromShuffleBoard, downSlot)
          .allowedClosedLoopError(allowedErrorDownFromShuffleBoard, downSlot);
      config.kF_Down = fDownFromShuffleBoard;
      config.kF_Up = fUpFromShuffleBoard;
      leaderMotor.configure(
          leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }
  // Sparkmax seems to have a bug where if you set PID values to .01 it actually sets .00099999999
  private boolean valuesActuallyDifferent(double val1, double val2) {
    return Math.abs(val1 - val2) > .000001;
  }
}
