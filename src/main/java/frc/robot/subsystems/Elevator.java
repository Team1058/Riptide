package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

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
  public double LEVEL1 = 4.75;
  //4.75
  public double LEVEL2 = 8.7;
  //8.7
  public double LEVEL3 = 15.6;
  //15.6
  public double LEVEL4 = 27.5;
  //26.9

  public double LEVELALGAEPROC = 1;
  public double LEVELALGAELOLLIPOP = 1;
  public double LEVELALGAE1 = 2.1;
  public double LEVELALGAE2 = 9.28;
  public double LEVELBARGE = 30.5;
  public double LEVELHP = 0.32;

  private final SparkFlex leaderMotor;
  private final SparkFlex followerMotor;
  private final RelativeEncoder elevatorEncoder;
  private SparkFlex followerMotor2 = null;
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
  private double rampRate = .3;
  private Config config;
  protected Double positionToHoldWhenNotSafeToMoveElevator;

  private final SysIdRoutine sysIdRoutine;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutAngle m_rotations = Rotations.mutable(0);
  private final MutAngularVelocity m_velocity = RPM.mutable(0);

  private static final ClosedLoopSlot downSlot = ClosedLoopSlot.kSlot0;
  private static final ClosedLoopSlot upSlot = ClosedLoopSlot.kSlot1;

  MechanismLigament2d m_elevatorL;
  MechanismLigament2d m_elevatorR;
  DCMotor elevatorMotors;
  SparkFlexSim elevatorMotorsSim;
  ElevatorSim elevatorSim;
  double kElevatorGearing = 25; // 25:1
  double kCarriageMass = 4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
  double kElevatorDrumRadius = 0.0328 / 2.0;
  double kMinElevatorHeightMeters = 0.922;
  double kMaxElevatorHeightMeters = 1.62;

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

      elevatorMotors = DCMotor.getNeoVortex(3);
      elevatorMotorsSim = new SparkFlexSim(leaderMotor, elevatorMotors);
      elevatorSim = new ElevatorSim(
          elevatorMotors,
          kElevatorGearing,
          kCarriageMass,
          kElevatorDrumRadius,
          kMinElevatorHeightMeters,
          kMaxElevatorHeightMeters,
          true,
          kMinElevatorHeightMeters,
          0.0,
          0.0);

      // the main mechanism object
      Mechanism2d elevatorL = new Mechanism2d(3, 3);
      Mechanism2d elevatorR = new Mechanism2d(3, 3);
      // the mechanism root node
       MechanismRoot2d rootL = elevatorL.getRoot("elevatorL", 1.25, 0 );
       MechanismRoot2d rootR = elevatorR.getRoot("elevatorR", 1.75, -0);

      
    Color8Bit red = new Color8Bit(Color.kRed);

    m_elevatorL = rootL.append(new MechanismLigament2d("elevatorLigamentL", 4, 80, 5, red ));
    m_elevatorR = rootR.append(new MechanismLigament2d("elevatorLigamentR", 4, 80, 5, red ));

    // post the mechanism to the dashboard
    SmartDashboard.putData("Mech2d", elevatorL);
    SmartDashboard.putData("Mech2d", elevatorR);
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
                setRequestedPosition(LEVELHP);
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
            sparkPIDController.setReference(requested, ControlType.kPosition, upSlot, config.kF_Up);
          } else {
            sparkPIDController.setReference(
                requested, ControlType.kPosition, downSlot, config.kF_Down);
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
    double lineLength = 2.449409 + (0.07223737 - 2.449409)/(1 + Math.pow(this.getCurrentPosition() / 15.07998, 1.825347));
    m_elevatorL.setLength(lineLength);
    m_elevatorR.setLength(lineLength);
  }

  public Command setRequestedPositionCommand(double position) {
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
    setRequestedPosition(getCurrentPosition());
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
  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    elevatorSim.setInput(leaderMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    elevatorSim.update(0.020);

    // Iterate the elevator and arm SPARK simulations
    elevatorMotorsSim.iterate(
        ((elevatorSim.getVelocityMetersPerSecond()
                    / (kElevatorDrumRadius * 2.0 * Math.PI))
                * kElevatorGearing)
            * 60.0,
        RobotController.getBatteryVoltage(),
        0.02);
  }
}
