package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms

  /// Name of the Canivore
  public static final String CAN_BUS_NAME = "swerve";

  /// CAN ID of the Piegeon 2.0 IMU. This is the same for all robots.
  public static final int PIGEON_ID = 61;

  // Odometry updates per second
  public static final Frequency ODOMETRY_UPDATE_FREQUENCY = Hertz.of(250);

  // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
  // This may need to be tuned to your individual robot
  public static final double COUPLE_RATIO = 3.5714285714285716;

  /// Mk4i L3 gearing ratios
  public static final double DRIVE_GEAR_RATIO = 6.122448979591837;
  public static final double STEER_GEAR_RATIO = 21.428571428571427;

  /// Theoretical maximum speeds based on module configuration
  public static final LinearVelocity MAX_LINEAR_SPEED = MetersPerSecond.of(4.9);

  /// Nominal wheel diameter
  public static final Distance WHEEEL_DIAMETER = Inches.of(4);

  /// Inverts for Kraken/Falcon module configuration
  public static final boolean INVERT_LEFT_DRIVE = false;
  public static final boolean INVERT_RIGHT_DRIVE = true;
  public static final boolean INVERT_STEER = true;
  public static final boolean INVERT_ENCODER = false;

  /// The closed-loop output type to use for the steer motors;
  /// This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType STEER_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType DRIVE_OUTPUT_TYPE = ClosedLoopOutputType.Voltage;

  /// These are only used for simulation
  public static final double STEER_INERTIA = 0.00001;
  public static final double DRIVE_INERTIA = 0.001;
  /// Simulated voltage necessary to overcome friction
  public static final double STEER_FRICTION_VOLTAGE = 0.25;
  public static final double DRIVE_FRICTION_VOLTAGE = 0.25;

  private AngularVelocity maxAngularVelocity;

  public PathPlannerPath path;
  public PPHolonomicDriveController ppDriveController;
  public PathConstraints pathConstraints;
  RobotConfig ppConfig;

  private Notifier simNotifier = null;
  private double lastSimTime;

  SendableChooser<SysIdRoutine> routineChooser;
  SendableChooser<Test> testChooser;

  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  private GenericEntry pitchPEntry;
  private GenericEntry pitchIEntry;
  private GenericEntry pitchDEntry;

  private GenericEntry yawPEntry;
  private GenericEntry yawIEntry;
  private GenericEntry yawDEntry;

  private GenericEntry rightpitchPEntry;
  private GenericEntry rightpitchIEntry;
  private GenericEntry rightpitchDEntry;

  private GenericEntry rightyawPEntry;
  private GenericEntry rightyawIEntry;
  private GenericEntry rightyawDEntry;

  private Field2d field2D;

  PIDController leftPitchPid;
  PIDController leftYawPid;
  PIDController rightPitchPid;
  PIDController rightYawPid;

  public static class Config {
    public enum Module {
      A(1, 21, 41, Rotations.of(0.455078125)),
      B(2, 22, 42, Rotations.of(0.0361328125)),
      C(3, 23, 43, Rotations.of(-0.252197265625)),
      D(4, 24, 44, Rotations.of(-0.03466796875)),
      E(5, 25, 45, Rotations.of(0.129639)),
      F(6, 26, 46, Rotations.of(-0.125732)),
      G(7, 27, 47, Rotations.of(-0.494141)),
      H(8, 28, 48, Rotations.of(0.372314)),
      I(9, 29, 49, Rotations.of(-0.376709)),
      J(10, 30, 50, Rotations.of(0.193848)),
      K(11, 31, 51, Rotations.of(0.420898)),
      W(12, 32, 52, Rotations.of(-0.229004)),
      N(13, 33, 53, Rotations.of(0)),
      O(14, 34, 54, Rotations.of(0)),
      P(15, 35, 55, Rotations.of(0)),
      Q(16, 36, 56, Rotations.of(0));

      public final int driveId;
      public final int steerId;
      public final int encoderId;
      // Absolute position reading of the encoder in calibration position (wheel at 45 degree angle
      // with bevel gear facing the steering motor)
      public final Angle encoderZeroPoint;

      private Module(int driveId, int steerId, int encoderId, Angle encoderZeroPoint) {
        this.driveId = driveId;
        this.steerId = steerId;
        this.encoderId = encoderId;
        this.encoderZeroPoint = encoderZeroPoint;
      }
    }

    public boolean shouldUsePIDForAlignment;

    public Distance sidelength = Inches.of(26);
    public Current slipCurrent = Amps.of(120.0);

    public Module frontLeftModule = Module.A;
    public Module backLeftModule = Module.B;
    public Module backRightModule = Module.C;
    public Module frontRightModule = Module.D;

    public double drivePGain = 0.35417;
    public double driveIGain = 0;
    public double driveDGain = 0;
    public double driveSGain = 0.070712;
    public double driveVGain = 0.12313;
    public double driveAGain = 0.008397;

    public double steerPGain = 100;
    public double steerIGain = 0;
    public double steerDGain = 0.5;
    public double steerSGain = 0.17159;
    public double steerVGain = 2.5744;
    public double steerAGain = 0.17104;

    public double servoSpeed = 0.2;
  }

  public static Drivetrain makeDrivetrain(Config config) {

    if (config == null) {
      return null;
    }

    final var driveGains = new Slot0Configs()
        .withKP(config.drivePGain)
        .withKI(config.driveIGain)
        .withKD(config.driveDGain)
        .withKS(config.driveSGain)
        .withKV(config.driveVGain)
        .withKA(config.driveAGain);
    final var steerGains = new Slot0Configs()
        .withKP(config.steerPGain)
        .withKI(config.steerIGain)
        .withKD(config.steerDGain)
        .withKS(config.steerSGain)
        .withKV(config.steerVGain)
        .withKA(config.steerAGain)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    final var driveInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true));

    final var steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            // Swerve azimuth does not require much torque output, so we can set a relatively low
            // stator current limit to help avoid brownouts without impacting performance.
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true));

    final var constants = new SwerveDrivetrainConstants()
        .withCANBusName(CAN_BUS_NAME)
        .withPigeon2Id(PIGEON_ID)
        .withPigeon2Configs(null);

    final var constantCreator = new SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
        .withSteerMotorGearRatio(STEER_GEAR_RATIO)
        .withWheelRadius(WHEEEL_DIAMETER.div(2))
        .withSlipCurrent(config.slipCurrent)
        .withSteerMotorGains(steerGains)
        .withDriveMotorGains(driveGains)
        .withSteerMotorClosedLoopOutput(STEER_OUTPUT_TYPE)
        .withDriveMotorClosedLoopOutput(DRIVE_OUTPUT_TYPE)
        .withSpeedAt12Volts(MAX_LINEAR_SPEED)
        .withSteerInertia(STEER_INERTIA)
        .withDriveInertia(DRIVE_INERTIA)
        .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
        .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE)
        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
        .withCouplingGearRatio(COUPLE_RATIO)
        .withDriveMotorInitialConfigs(driveInitialConfigs)
        .withSteerMotorInitialConfigs(steerInitialConfigs);

    final Distance halfTrackWidth = config.sidelength.minus(Inches.of(5.25)).div(2);

    final var frontLeft = constantCreator.createModuleConstants(
        config.frontLeftModule.steerId,
        config.frontLeftModule.driveId,
        config.frontLeftModule.encoderId,
        config.frontLeftModule.encoderZeroPoint.plus(Degrees.of(135)).unaryMinus(),
        halfTrackWidth,
        halfTrackWidth,
        INVERT_LEFT_DRIVE,
        INVERT_STEER,
        INVERT_ENCODER);
    final var backLeft = constantCreator.createModuleConstants(
        config.backLeftModule.steerId,
        config.backLeftModule.driveId,
        config.backLeftModule.encoderId,
        config.backLeftModule.encoderZeroPoint.plus(Degrees.of(45)).unaryMinus(),
        halfTrackWidth.unaryMinus(),
        halfTrackWidth,
        INVERT_LEFT_DRIVE,
        INVERT_STEER,
        INVERT_ENCODER);
    final var backRight = constantCreator.createModuleConstants(
        config.backRightModule.steerId,
        config.backRightModule.driveId,
        config.backRightModule.encoderId,
        config.backRightModule.encoderZeroPoint.plus(Degrees.of(135)).unaryMinus(),
        halfTrackWidth.unaryMinus(),
        halfTrackWidth.unaryMinus(),
        INVERT_RIGHT_DRIVE,
        INVERT_STEER,
        INVERT_ENCODER);
    final var frontRight = constantCreator.createModuleConstants(
        config.frontRightModule.steerId,
        config.frontRightModule.driveId,
        config.frontRightModule.encoderId,
        config.frontRightModule.encoderZeroPoint.plus(Degrees.of(45)).unaryMinus(),
        halfTrackWidth,
        halfTrackWidth.unaryMinus(),
        INVERT_RIGHT_DRIVE,
        INVERT_STEER,
        INVERT_ENCODER);

    return new Drivetrain(
        constants, halfTrackWidth, config, frontLeft, frontRight, backLeft, backRight);
  }

  public enum Test {
    QUASISTATIC_FORWARD,
    QUASISTATIC_REVERSE,
    DYNAMIC_FORWARD,
    DYNAMIC_REVERSE;
  }

  /** Create a drivetrain */
  private Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      Distance halfTrackWidth,
      Config config,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        driveTrainConstants,
        ODOMETRY_UPDATE_FREQUENCY.in(Hertz),
        modules);

    final double trackRadius_m = Math.hypot(halfTrackWidth.in(Meters), halfTrackWidth.in(Meters));
    this.maxAngularVelocity =
        RadiansPerSecond.of(MAX_LINEAR_SPEED.in(MetersPerSecond) / trackRadius_m);
    try {
      ppConfig = RobotConfig.fromGUISettings();
      System.out.println("Loaded pp config json");
    } catch (Exception e) {
      e.printStackTrace();
    }

    ppDriveController =
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            // new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
            // new PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
            new PIDConstants(4, 0.0, 0.0), // Translation PID constants
            new PIDConstants(4, 0.0, 0.0) // Rotation PID constants

            );
    pathConstraints = new PathConstraints(
        MAX_LINEAR_SPEED,
        MetersPerSecondPerSecond.of(3),
        maxAngularVelocity,
        RadiansPerSecondPerSecond.of(4 * Math.PI));
    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> this.setControl(new SwerveRequest.ApplyRobotSpeeds()
            .withSpeeds(speeds)
            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
        // optionally outputs individual module feedforwards
        ppDriveController,
        ppConfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );

    // Prime some kind of Jave cache (the docs say to do this).
    PathfindingCommand.warmupCommand().schedule();

    // Setup callbacks to log internal pathplanner state
    PathPlannerLogging.setLogActivePathCallback((trajectory) -> Logger.recordOutput(
        "Drivetrain/ActivePath", trajectory.toArray(new Pose2d[trajectory.size()])));
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> Logger.recordOutput("Drivetrain/TargetPose", pose));

    if (Utils.isSimulation()) {
      startSimThread();
    }

    initializeShuffleboardEntries();

    var sysidTab = Shuffleboard.getTab("SysID");

    routineChooser = new SendableChooser<SysIdRoutine>();
    testChooser = new SendableChooser<Test>();

    routineChooser.setDefaultOption("None", null);
    routineChooser.addOption("Drive", sysIdRoutineDrive);
    routineChooser.addOption("Steer", sysIdRoutineSteer);
    routineChooser.addOption("Heading", sysIdRoutineHeading);

    testChooser.setDefaultOption("Quasistatic Forward", Test.QUASISTATIC_FORWARD);
    testChooser.addOption("Quasistatic Reverse", Test.QUASISTATIC_REVERSE);
    testChooser.addOption("Dynamic Forward", Test.DYNAMIC_FORWARD);
    testChooser.addOption("Dynamic Reverse", Test.DYNAMIC_REVERSE);

    sysidTab.add("Routine", routineChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    sysidTab.add("Test", testChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

    sysidTab.addString("Drivetrain Command", () -> {
      if (this.getCurrentCommand() != null) {
        return this.getCurrentCommand().getName();
      } else return "none";
    });

    field2D = new Field2d();
    SmartDashboard.putData("Field", field2D);

    SmartDashboard.putData("SwerveDrive", builder -> {
      builder.setSmartDashboardType("SwerveDrive");
      builder.addDoubleProperty(
          "Front Left Angle", () -> getState().ModuleStates[0].angle.getRadians(), null);
      builder.addDoubleProperty(
          "Front Left Velocity", () -> getState().ModuleStates[0].speedMetersPerSecond, null);
      builder.addDoubleProperty(
          "Front Right Angle", () -> getState().ModuleStates[1].angle.getRadians(), null);
      builder.addDoubleProperty(
          "Front Right Velocity", () -> getState().ModuleStates[1].speedMetersPerSecond, null);
      builder.addDoubleProperty(
          "Rear Left Angle", () -> getState().ModuleStates[2].angle.getRadians(), null);
      builder.addDoubleProperty(
          "Rear Left Velocity", () -> getState().ModuleStates[2].speedMetersPerSecond, null);
      builder.addDoubleProperty(
          "Rear Right Angle", () -> getState().ModuleStates[3].angle.getRadians(), null);
      builder.addDoubleProperty(
          "Rear Right Velocity", () -> getState().ModuleStates[3].speedMetersPerSecond, null);

      builder.addDoubleProperty("Robot Angle", () -> getPose().getRotation().getRadians(), null);
    });

    leftPitchPid = new PIDController(0.3, 0, 0.0005); // Tune these values
    leftYawPid = new PIDController(0.2, 0, 0.002); // Tune these values
    rightPitchPid = new PIDController(0.3, 0, 0.0005); // Tune these values
    rightYawPid = new PIDController(0.2, 0, 0.002); // Tune these values
  }

  public Pose2d getPose() {
    return this.getState().Pose;
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return this.getState().Speeds;
  }

  public AngularVelocity getMaxAngularVelocity() {
    return maxAngularVelocity;
  }

  public Command runSysIdTest() {
    return Commands.defer(() -> this.getSysIdCommand(), Set.of(this)).withName("Run Sys Id Test");
  }

  public Command resetPerspective() {
    return Commands.runOnce(() -> setOperatorPerspectiveForward(getPose().getRotation()), this)
        .withName("Reset Perspective");
  }

  public Trigger getTippingTrigger() {

    return new Trigger(() -> ((Math.abs(getRotation3d().getMeasureX().baseUnitMagnitude()) > 0.10)
        || (Math.abs(getRotation3d().getMeasureY().baseUnitMagnitude()) > 0.15)));
  }

  public Trigger getAlignedTrigger(
      double targetPitch,
      double targetYaw,
      DoubleSupplier currentPitch,
      DoubleSupplier currentYaw) {
    Logger.recordOutput("currentPitch", currentPitch);
    Logger.recordOutput("currentYaw", currentYaw);
    return new Trigger(() -> ((Math.abs(targetYaw - currentYaw.getAsDouble()) < 10000.0)
        && (Math.abs(targetPitch - currentPitch.getAsDouble()) < 10000.0)));
  }

  private Command getSysIdCommand() {
    var routine = routineChooser.getSelected();
    var test = testChooser.getSelected();

    System.out.println("Trying to run sysid test");
    if (routine == null) {
      System.out.println("Routine is null");
      return Commands.none();
    }
    switch (test) {
      case QUASISTATIC_FORWARD:
        return routine.quasistatic(Direction.kForward);
      case QUASISTATIC_REVERSE:
        return routine.quasistatic(Direction.kReverse);
      case DYNAMIC_FORWARD:
        return routine.dynamic(Direction.kForward);
      case DYNAMIC_REVERSE:
        return routine.dynamic(Direction.kReverse);
      default:
        return Commands.none();
    }
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get())).withName("Drive apply request");
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - lastSimTime;
      lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }

  public Command getPathFindingCommand(Pose2d endPose) {
    return AutoBuilder.pathfindToPose(endPose, pathConstraints)
        .withName("Get Path Finding Command");
  }

  public Command alignLeftPitchToTagCommand(
      double targetPitch, DoubleSupplier currentPitch, boolean shouldUsePID) {
    if (shouldUsePID) {
      return getAlignPitchToTagCommandViaPID(targetPitch, currentPitch, leftPitchPid);
    }
    return getAlignDirectionToTagCommandViaBangBang(targetPitch, currentPitch)
        .withName("Align Left Pitch To Tag Command");
  }

  public Command alignRightPitchToTagCommand(
      double targetPitch, DoubleSupplier currentPitch, boolean shouldUsePID) {
    if (shouldUsePID) {
      return getAlignPitchToTagCommandViaPID(targetPitch, currentPitch, rightPitchPid);
    }
    return getAlignDirectionToTagCommandViaBangBang(targetPitch, currentPitch)
        .withName("Align Right Pitch To Tag Command");
  }

  private Command getAlignPitchToTagCommandViaPID(
      double targetPitch, DoubleSupplier currentPitch, PIDController pitchPID) {
    SwerveRequest.RobotCentric drive =
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

    pitchPID.setTolerance(0.5); // degrees

    return new FunctionalCommand(
        () -> {},
        () -> {
          double xVelocity = -1 * pitchPID.calculate(currentPitch.getAsDouble(), targetPitch);
          xVelocity = MathUtil.clamp(xVelocity, -.3, 0.3);

          if (currentPitch.getAsDouble() == 1058.0) {
            xVelocity = 0;
          }

          this.setControl(drive
              .withVelocityX(xVelocity)
              .withVelocityY(0)
              .withRotationalRate(0)); // update rotational rate to the PID controller if needed
        },
        interrupted ->
            this.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)),
        pitchPID::atSetpoint, // add rotation in if needed
        this);
  }

  public Command alignLeftYawToTagCommand(
      double targetYaw, DoubleSupplier currentYaw, boolean shouldUsePID) {
    if (shouldUsePID) {
      return getAlignYawToTagCommandViaPID(targetYaw, currentYaw, leftYawPid);
    }
    return getAlignDirectionToTagCommandViaBangBang(targetYaw, currentYaw)
        .withName("Align Left Yaw To Tag Command");
  }

  public Command alignRightYawToTagCommand(
      double targetYaw, DoubleSupplier currentYaw, boolean shouldUsePID) {
    if (shouldUsePID) {
      return getAlignYawToTagCommandViaPID(targetYaw, currentYaw, rightYawPid)
          .withName("Align Right Yaw To Tag Command");
    }
    return getAlignDirectionToTagCommandViaBangBang(targetYaw, currentYaw)
        .withName("Align Right Yaw To Tag Command");
  }

  public Command getAlignYawToTagCommandViaPID(
      double targetYaw, DoubleSupplier currentYaw, PIDController yawPID) {
    SwerveRequest.RobotCentric drive =
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

    yawPID.setTolerance(0.5); // degrees

    return new FunctionalCommand(
            () -> {},
            () -> {
              double yVelocity = -1 * yawPID.calculate(currentYaw.getAsDouble(), targetYaw);
              yVelocity = MathUtil.clamp(yVelocity, -.3, 0.3);

              if (currentYaw.getAsDouble() == 1058.0) {
                yVelocity = 0;
              }

              this.setControl(drive
                  .withVelocityX(0)
                  .withVelocityY(yVelocity)
                  .withRotationalRate(0)); // update rotational rate to the PID controller if needed
            },
            interrupted ->
                this.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)),
            yawPID::atSetpoint, // add rotation in if needed
            this)
        .withName("Get Align Yaw To Tag Command Via PID");
  }

  public Command getAlignDirectionToTagCommandViaBangBang(
      double targetValue, DoubleSupplier currentValue) {
    SwerveRequest.RobotCentric drive =
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

    double currentTolerance = 0.5; // degrees

    return new FunctionalCommand(
            () -> {},
            () -> {
              double yVelocity = 0.0;
              if (currentValue.getAsDouble() == 1058.0) {
                yVelocity = 0;
              } else if (currentValue.getAsDouble() < targetValue) {
                yVelocity = -0.1;
              } else if (currentValue.getAsDouble() > targetValue) {
                yVelocity = 0.1;
              }

              this.setControl(drive
                  .withVelocityX(0)
                  .withVelocityY(yVelocity)
                  .withRotationalRate(0)); // update rotational rate to the PID controller if needed
            },
            interrupted ->
                this.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)),
            () -> Math.abs(currentValue.getAsDouble() - targetValue)
                < currentTolerance, // add rotation in if needed
            this)
        .withName("Get Align Direction To Tag Command Via Bang Bang");
  }

  private void initializeShuffleboardEntries() {
    pitchPEntry = Shuffleboard.getTab("PID")
        .add("Left Pitch P", 0.1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    pitchIEntry = Shuffleboard.getTab("PID")
        .add("Left Pitch I", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    pitchDEntry = Shuffleboard.getTab("PID")
        .add("Left Pitch D", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    yawPEntry = Shuffleboard.getTab("PID")
        .add("Left Yaw P", 0.1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    yawIEntry = Shuffleboard.getTab("PID")
        .add("Left Yaw I", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    yawDEntry = Shuffleboard.getTab("PID")
        .add("Left Yaw D", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    rightpitchPEntry = Shuffleboard.getTab("PID")
        .add("Right Pitch P", 0.1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    rightpitchIEntry = Shuffleboard.getTab("PID")
        .add("Right Pitch I", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    rightpitchDEntry = Shuffleboard.getTab("PID")
        .add("Right Pitch D", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

    rightyawPEntry = Shuffleboard.getTab("PID")
        .add("Right Yaw P", 0.1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    rightyawIEntry = Shuffleboard.getTab("PID")
        .add("Right Yaw I", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    rightyawDEntry = Shuffleboard.getTab("PID")
        .add("Right Yaw D", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
  }

  public Command makeGoToCommand(
      Rotation2d endHeading, LinearVelocity endVelocity, List<Pose2d> poses) {
    try {
      // Use final pose getter from field map
      PathPlannerPath path = getPath(endHeading, endVelocity, poses);
      FollowPathCommand pathFollowCommand = new FollowPathCommand(
          path,
          this::getPose,
          this::getCurrentSpeeds,
          (speeds, feedforwards) -> this.setControl(new SwerveRequest.ApplyRobotSpeeds()
              .withSpeeds(speeds)
              .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
              .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          ppDriveController,
          ppConfig,
          () -> {
            return false;
          },
          this);

      return pathFollowCommand.withName("GoTo");
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to create GoTo command" + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public Command makeGoToCommand(
      Rotation2d endHeading, LinearVelocity endVelocity, Pose2d... poses) {
    return makeGoToCommand(endHeading, endVelocity, Arrays.asList(poses));
  }

  // Make command to go to end pose with
  public Command makeGoToCommandWithMaxMPS(
      Rotation2d endHeading,
      LinearVelocity startVelocity,
      LinearVelocity endVelocity,
      double maxLinearSpeedMPS,
      List<Pose2d> poses) {
    try {

      PathPlannerPath path =
          getPathWithMaxMPS(endHeading, startVelocity, endVelocity, maxLinearSpeedMPS, poses);

      FollowPathCommand pathFollowCommand = new FollowPathCommand(
          path,
          this::getPose,
          this::getCurrentSpeeds,
          (speeds, feedforwards) -> this.setControl(new SwerveRequest.ApplyRobotSpeeds()
              .withSpeeds(speeds)
              .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
              .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          ppDriveController,
          ppConfig,
          () -> {
            return false;
          },
          this);

      return pathFollowCommand.withName("GoTo");
    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to create GoTo command" + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public Command makeGoToCommandWithMaxMPS(
      Rotation2d endHeading,
      LinearVelocity startVelocity,
      LinearVelocity endVelocity,
      double maxLinearSpeedMPS,
      Pose2d... poses) {
    return makeGoToCommandWithMaxMPS(
        endHeading, startVelocity, endVelocity, maxLinearSpeedMPS, Arrays.asList(poses));
  }

  public Command makeGoToPoseCommand(Pose2d endPose, Rotation2d heading, Double endVelocity) {
    return makeGoToCommand(heading, MetersPerSecond.of(endVelocity), endPose);
  }

  /** Generate a path from the current robot position through the requested set of poses.
   *
   * The path will start at the robot's current position and take into account its current velocity and direction of
   * travel. The rotoation component of the provided poses referese to the robot's direction of travel, not the robot's
   * heading. Use the 'endHeading' paramter to set a desired end heading for the robot.
   *
   * @param endHeading Desired robot heading at the end of the path
   * @param endVelocity Desired linear velocity (in the directon of the last pose) at the end of the path.
   * @param poses List of poses (waypoints) to pass through along the path. The last pose in this list is the desired
   *  end point. The rotation component of these poses is the direction of travel, not the robot heading.
   * @return The generated path.
   */
  public PathPlannerPath getPath(
      Rotation2d endHeading, LinearVelocity endVelocity, List<Pose2d> poses) {
    double directionOfTravelAngle = 0;
    double startingVelocity = Math.hypot(
        this.getState().Speeds.vyMetersPerSecond, this.getState().Speeds.vxMetersPerSecond);
    var currentPose = this.getPose();
    if (startingVelocity > 0.1) {
      directionOfTravelAngle = Math.atan2(
          this.getState().Speeds.vyMetersPerSecond, this.getState().Speeds.vxMetersPerSecond);
    }
    Rotation2d directionOfTravel = new Rotation2d(directionOfTravelAngle);

    Pose2d startPose = new Pose2d(currentPose.getX(), currentPose.getY(), directionOfTravel);

    // Waypoints generated from current, any intermediate poses, and endpose
    var all_poses = new ArrayList<Pose2d>(1 + poses.size());
    all_poses.add(startPose);
    all_poses.addAll(poses);
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(all_poses);

    path = new PathPlannerPath(
        waypoints,
        pathConstraints,
        new IdealStartingState(startingVelocity, currentPose.getRotation()),
        new GoalEndState(endVelocity, endHeading));
    path.preventFlipping = true;
    return path;
  }

  public PathPlannerPath getPathWithMaxMPS(
      Rotation2d endHeading,
      LinearVelocity startVelocity,
      LinearVelocity endVelocity,
      double maxLinearSpeedMPS,
      List<Pose2d> poses) {
    var currentPose = this.getPose();
    double directionOfTravelAngle =
        currentPose.getRotation().rotateBy(Rotation2d.k180deg).getDegrees();
    double startingVelocity = startVelocity.in(MetersPerSecond);
    if (startingVelocity > 0.1) {
      directionOfTravelAngle = Math.atan2(
          this.getState().Speeds.vyMetersPerSecond, this.getState().Speeds.vxMetersPerSecond);
    }
    Rotation2d directionOfTravel = new Rotation2d(directionOfTravelAngle);

    Pose2d startPose = new Pose2d(currentPose.getX(), currentPose.getY(), directionOfTravel);
    Logger.recordOutput("StartPoseForPathPlanner", startPose);

    // Waypoints generated from current, any intermediate poses, and endpose
    var all_poses = new ArrayList<Pose2d>(1 + poses.size());
    all_poses.add(startPose);
    all_poses.addAll(poses);
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(all_poses);

    PathConstraints pathConstraints = new PathConstraints(
        MetersPerSecond.of(maxLinearSpeedMPS),
        MetersPerSecondPerSecond.of(3),
        maxAngularVelocity,
        RadiansPerSecondPerSecond.of(4 * Math.PI));

    path = new PathPlannerPath(
        waypoints,
        pathConstraints,
        new IdealStartingState(startingVelocity, currentPose.getRotation()),
        new GoalEndState(endVelocity, endHeading));
    path.preventFlipping = true;
    return path;
  }

  public PathPlannerPath getPath(
      Rotation2d endHeading, LinearVelocity endVelocity, Pose2d... poses) {
    return getPath(endHeading, endVelocity, Arrays.asList(poses));
  }

  public PathPlannerPath getPathToPose(Pose2d endPose, Rotation2d heading, Double endVelocity) {
    return getPath(heading, MetersPerSecond.of(endVelocity), endPose);
  }
  // defer command that creates path following command than returns it.

  public void setAllMotorsBrake() {
    for (int i = 0; i < 4; i++) {
      this.getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
      this.getModule(i).getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
    }
  }

  public void setAllMotorsCoast() {
    for (int i = 0; i < 4; i++) {
      this.getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
      this.getModule(i).getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public void periodic() {
    /* Periodically try to apply the operator perspective */
    /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
    /* This allows us to correct the perspective in case the robot code restarts mid-match */
    /* Otherwise, only check and apply the operator perspective if the DS is disabled */
    /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent((allianceColor) -> {
        this.setOperatorPerspectiveForward(
            allianceColor == Alliance.Red
                ? Rotation2d.fromDegrees(180)
                : Rotation2d.fromDegrees(0));
        hasAppliedOperatorPerspective = true;
      });
    }

    field2D.setRobotPose(getPose());

    var state = getState();

    Logger.recordOutput("Drivetrain/Pose", getPose());
    Logger.recordOutput("Drivetrain/ChassisSpeeds", getCurrentSpeeds());
    Logger.recordOutput("Drivetrain/ModuleStates", state.ModuleStates);
    Logger.recordOutput("Drivetrain/ModuleTargets", state.ModuleTargets);
    Logger.recordOutput("Drivetrain/Pitch", getRotation3d().getY());
    Logger.recordOutput("Drivetrain/Roll", getRotation3d().getX());
  }

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation driveCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation headingCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine sysIdRoutineDrive = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> setControl(driveCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(7), // Use dynamic voltage of 7 V
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> setControl(steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine sysIdRoutineHeading = new SysIdRoutine(
      new SysIdRoutine.Config(
          /* This is in radians per second², but SysId only supports "volts per second" */
          Volts.of(Math.PI / 6).per(Second),
          /* This is in radians per second, but SysId only supports "volts" */
          Volts.of(Math.PI),
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> {
            /* output is actually radians per second, but SysId only supports "volts" */
            setControl(headingCharacterization.withRotationalRate(output.in(Volts)));
            /* also log the requested output for SysId */
            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
          },
          null,
          this));

  public void testPeriodic() {

    double leftpPitchFromShuffleBoard = pitchPEntry.getDouble(0.1);
    double leftiPitchFromShuffleBoard = pitchIEntry.getDouble(0);
    double leftdPitchFromShuffleBoard = pitchDEntry.getDouble(0);

    double leftcurrentPitchP = leftPitchPid.getP();
    double leftcurrentPitchI = leftPitchPid.getI();
    double leftcurrentPitchD = leftPitchPid.getD();

    double leftpYawFromShuffleBoard = yawPEntry.getDouble(0.1);
    double leftiYawFromShuffleBoard = yawIEntry.getDouble(0);
    double leftdYawFromShuffleBoard = yawDEntry.getDouble(0);

    double leftcurrentYawP = leftYawPid.getP();
    double leftcurrentYawI = leftYawPid.getI();
    double leftcurrentYawD = leftYawPid.getD();

    double rightpPitchFromShuffleBoard = rightpitchPEntry.getDouble(0.1);
    double rightiPitchFromShuffleBoard = rightpitchIEntry.getDouble(0);
    double rightdPitchFromShuffleBoard = rightpitchDEntry.getDouble(0);

    double rightcurrentPitchP = rightPitchPid.getP();
    double rightcurrentPitchI = rightPitchPid.getI();
    double rightcurrentPitchD = rightPitchPid.getD();

    double rightpYawFromShuffleBoard = rightyawPEntry.getDouble(0.1);
    double rightiYawFromShuffleBoard = rightyawIEntry.getDouble(0);
    double rightdYawFromShuffleBoard = rightyawDEntry.getDouble(0);

    double rightcurrentYawP = rightYawPid.getP();
    double rightcurrentYawI = rightYawPid.getI();
    double rightcurrentYawD = rightYawPid.getD();

    if (valuesActuallyDifferent(leftpPitchFromShuffleBoard, leftcurrentPitchP)
        || valuesActuallyDifferent(leftiPitchFromShuffleBoard, leftcurrentPitchI)
        || valuesActuallyDifferent(leftdPitchFromShuffleBoard, leftcurrentPitchD)) {

      leftPitchPid.setPID(
          leftpPitchFromShuffleBoard, leftiPitchFromShuffleBoard, leftdPitchFromShuffleBoard);
    }
    if (valuesActuallyDifferent(leftpYawFromShuffleBoard, leftcurrentYawP)
        || valuesActuallyDifferent(leftiYawFromShuffleBoard, leftcurrentYawI)
        || valuesActuallyDifferent(leftdYawFromShuffleBoard, leftcurrentYawD)) {

      leftYawPid.setPID(
          leftpYawFromShuffleBoard, leftiYawFromShuffleBoard, leftdYawFromShuffleBoard);
    }
    if (valuesActuallyDifferent(rightpPitchFromShuffleBoard, rightcurrentPitchP)
        || valuesActuallyDifferent(rightiPitchFromShuffleBoard, rightcurrentPitchI)
        || valuesActuallyDifferent(rightdPitchFromShuffleBoard, rightcurrentPitchD)) {

      rightPitchPid.setPID(
          rightpPitchFromShuffleBoard, rightiPitchFromShuffleBoard, rightdPitchFromShuffleBoard);
    }
    if (valuesActuallyDifferent(rightpYawFromShuffleBoard, rightcurrentYawP)
        || valuesActuallyDifferent(rightiYawFromShuffleBoard, rightcurrentYawI)
        || valuesActuallyDifferent(rightdYawFromShuffleBoard, rightcurrentYawD)) {

      rightYawPid.setPID(
          rightpYawFromShuffleBoard, rightiYawFromShuffleBoard, rightdYawFromShuffleBoard);
    }
  }

  private boolean valuesActuallyDifferent(double val1, double val2) {
    return Math.abs(val1 - val2) > .000001;
  }
}
