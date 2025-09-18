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
import frc.robot.utils.PidConsumer;
import frc.robot.utils.TunableConstant;
import frc.robot.utils.TunablePID;

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
  //DON'T go beyond 31.25

  public TunableConstant LEVEL1;
  public TunableConstant LEVEL2;
  public TunableConstant LEVEL3;
  public TunableConstant LEVEL4;
  public TunableConstant LEVEL4CORALDISENGAGE;

  public TunableConstant LEVELALGAEPROC;
  public TunableConstant LEVELALGAEFLOOR;
  public TunableConstant LEVELALGAELOLLIPOP;
  public TunableConstant LEVELALGAE1;
  public TunableConstant LEVELALGAE2;
  public TunableConstant LEVELBARGE;
  public TunableConstant LEVELHP;

  private final SparkFlex leaderMotor;
  private final SparkFlex followerMotor;
  private final RelativeEncoder elevatorEncoder;
  private SparkFlex followerMotor2 = null;
  private SparkFlexConfig leaderConfig;
  private SparkFlexConfig followerConfig;
  private SparkFlexConfig follower2Config;
  private SparkClosedLoopController sparkPIDController;
  protected double requestedPosition;


  private int stallLimit = 90;
  private int freeLimit = 90;
  private double rampRate = .3;
  private Config config;
  protected Double positionToHoldWhenNotSafeToMoveElevator;

  private final SysIdRoutine sysIdRoutine;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutAngle m_rotations = Rotations.mutable(0);
  private final MutAngularVelocity m_velocity = RPM.mutable(0);

  private static final ClosedLoopSlot downSlot = ClosedLoopSlot.kSlot0;
  private static final ClosedLoopSlot upSlot = ClosedLoopSlot.kSlot1;

  private TunablePID elevatorPidUp, elevatorPidDown;
  public Elevator(Config config) {

    this.config = config;
    leaderConfig = new SparkFlexConfig();
    followerConfig = new SparkFlexConfig();
    follower2Config = new SparkFlexConfig();
    leaderMotor = new SparkFlex(config.leaderMotorId, MotorType.kBrushless);
    followerMotor = new SparkFlex(config.followerMotorId, MotorType.kBrushless);
    elevatorPidUp = new TunablePID("elevatorPID_Up", config.kP_Up, config.kI_Up, config.kD_Up, config.kF_Up);
    elevatorPidDown = new TunablePID("elevatorPID_Down", config.kP_Down, config.kI_Down, config.kD_Down, config.kF_Down);
    LEVELALGAEPROC = new TunableConstant("/ElevatorLevels/AlgaeProcessor",1);
    LEVELALGAELOLLIPOP = new TunableConstant("/ElevatorLevel/AlgaeLollipop",1);
    LEVELALGAEFLOOR = new TunableConstant("/ElevatorLevel/AlgaeFloor",0.1);
    LEVELALGAE1 = new TunableConstant( "/ElevatorLevels/AlgaeLevel1", 4.6);
    LEVELALGAE2 = new TunableConstant("/ElevatorLevels/AlgaeLevel2", 11);
    LEVELBARGE = new TunableConstant("/ElevatorLevels/AlgaeBarge", 30.5);
    LEVELHP = new TunableConstant("/ElevatorLevels/CoralHP", 0.32);
    LEVEL1 = new TunableConstant("/ElevatorLevels/CoralLevel1", 4.75);
    LEVEL2 = new TunableConstant("/ElevatorLevels/CoralLevel2", 8.7);
    LEVEL3 = new TunableConstant("/ElevatorLevels/CoralLevel3", 15.6);
    LEVEL4 = new TunableConstant("/ElevatorLevels/CoralLevel4", 27.5);
    LEVEL4CORALDISENGAGE = new TunableConstant("/ElevatorLevels/CoralLevel4CoralDisengage", 29.5);
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
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

    leaderConfig
        .closedLoop
        .pid(config.kP_Up, config.kI_Up, config.kD_Up, upSlot).outputRange(-0.75, .75, upSlot)
        .pid(config.kP_Down, config.kI_Down, config.kD_Down, downSlot).outputRange(-0.4, .4, downSlot)
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

    requestedPosition = 0;
    sparkPIDController = leaderMotor.getClosedLoopController();
    elevatorEncoder = leaderMotor.getEncoder();

    positionToHoldWhenNotSafeToMoveElevator = null;

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


  public void setRequestedPosition(DoubleSupplier requested) {
    this.requestedPosition = MathUtil.clamp(requested.getAsDouble(), config.lowerLimit, config.upperLimit);
  }

  public void setRequestedPositionToCurrentPosition() {
    this.setRequestedPosition(this::getCurrentPosition);
  }

  public boolean currentPositionAtTarget(double target) {
    return Math.abs(this.getCurrentPosition() - target) < 0.2;
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

  // public Command manualDriveCommand(DoubleSupplier speedSupplier) {
  //   return run(() -> {
  //         double positionDelta = (speedSupplier.getAsDouble() * -1) * 2;
  //         double requested = this.getCurrentPosition() + positionDelta;
  //         this.setRequestedPosition(requested);

  //         if (requested > getCurrentPosition()) {
  //           sparkPIDController.setReference(requested, ControlType.kPosition, upSlot, config.kF_Up);
  //         } else {
  //           sparkPIDController.setReference(
  //               requested, ControlType.kPosition, downSlot, config.kD_Down);
  //         }
  //       })
  //       .withName("manualDrive")
  //       .withName("Manul Drive Command");
  // }

  public Command resetElevatorCommand() {
    return new FunctionalCommand(
            () -> {
              leaderConfig.softLimit.reverseSoftLimitEnabled(false);
              leaderMotor.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            () -> leaderMotor.set(-.1),
            interrupted -> {
              if (!interrupted) {
                leaderMotor.set(0);
                elevatorEncoder.setPosition(0);
                requestedPosition = 0;
                setRequestedPosition(LEVELHP::getAndUpdate);
              }
              leaderConfig.softLimit.reverseSoftLimitEnabled(true);
              leaderMotor.configure(
                leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                
            },
            () -> leaderMotor.getOutputCurrent() >= 30,
            this)
        .withName("Reset Elevator Command");
  }

  public boolean isAtPosition(double position) {
    return Math.abs(getCurrentPosition() - position) < config.allowedError_Up;
  }

  public Trigger isAtPositionTrigger(double position) {
      return new Trigger(
        ()-> isAtPosition(position));
  }

  public double getCurrentPosition() {
    return elevatorEncoder.getPosition();
  }

  public Command goToRequestedPositionCommand() {

    return runOnce(() -> {

          double requested;

            requested = getRequestedPosition();

          // if (requested- getCurrentPosition() < 0.1)
          if (requested > getCurrentPosition()) {
            sparkPIDController.setReference(requested, ControlType.kPosition, upSlot, elevatorPidUp.getKF());
          } else {
            sparkPIDController.setReference(
                requested, ControlType.kPosition, downSlot, elevatorPidDown.getKF());
          }
        })
        .withName("goToRequestedPosition");

  }

  public double getRequestedPosition() {
    return requestedPosition;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Elevator Encoder Position", getCurrentPosition());
    Logger.recordOutput("Elevator Requested Position", getRequestedPosition());
    Logger.recordOutput("Leader Motor Output", leaderMotor.getAppliedOutput());
    Logger.recordOutput("Elevator Speed", leaderMotor.getEncoder().getVelocity());
    Logger.recordOutput("Elevator Current", leaderMotor.getOutputCurrent());
  }

  public Command setRequestedPositionCommand(DoubleSupplier position) {
    return runOnce(
      () -> this.setRequestedPosition(position))
    .withName("setRequestedPosition")
    .andThen(goToRequestedPositionCommand());
  }

  public Command setRequestedToCurrentPositionCommand() {
    return runOnce(
      () -> holdPosition())
    .withName("setRequestedPositionFromSupplier");
  }

  public void holdPosition() {
    setRequestedPosition(this::getCurrentPosition);
  }
  private void updateElevatorPid(double kP, double kI, double kD, ClosedLoopSlot closedLoop){
    leaderConfig
    .closedLoop
    .pid(kP, kI, kD, closedLoop);
    leaderMotor.configure(
      leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  public void testPeriodic() {
    elevatorPidDown.updatePID((kP, kI, kD) -> updateElevatorPid(kP, kI, kD, downSlot));
    elevatorPidUp.updatePID((kP, kI, kD) -> updateElevatorPid(kP, kI, kD, upSlot));
    
  }
  public void testInit() {
    elevatorPidDown.setTuningMode(true);
    elevatorPidUp.setTuningMode(true);
    LEVELALGAELOLLIPOP.setTuningMode(true);
    LEVELALGAEPROC.setTuningMode(true);
    LEVELALGAEFLOOR.setTuningMode(true);
    LEVELALGAE1.setTuningMode(true);
    LEVELALGAE2.setTuningMode(true);
    LEVELBARGE.setTuningMode(true);
    LEVELHP.setTuningMode(true);
    LEVEL1.setTuningMode(true);
    LEVEL2.setTuningMode(true);
    LEVEL3.setTuningMode(true);
    LEVEL4.setTuningMode(true);
  }
}
