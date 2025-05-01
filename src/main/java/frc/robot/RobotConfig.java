package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Processor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import java.util.HashMap;
import java.util.Map;

public class RobotConfig {
  String name;
  int driverControllerPort;
  int operatorControllerPort;
  boolean hasSubsystems;
  boolean hasProcessor;
  boolean hasClimber;
  boolean hasShooter;
  boolean hasIntake;
  Elevator.Config elevatorConfig;
  Shooter.Config shooterConfig;
  Intake.Config intakeConfig;
  Processor.Config processorConfig;
  Climber.Config climberConfig;
  Drivetrain.Config drivetrainConfig;
  Vision.Config visionConfig;

  static Map<RoboRio, RobotConfig> knownConfigs;

  static {
    knownConfigs = new HashMap<RoboRio, RobotConfig>();
    knownConfigs.put(RoboRio.GAMMA, getProtoBoardConfig());
    knownConfigs.put(RoboRio.ALPHA, getV1Config());
    knownConfigs.put(RoboRio.BETA, getV2Config());
    knownConfigs.put(RoboRio.DELTA, getV3Config());
    knownConfigs.put(RoboRio.EPSILON, getV3Config());
  }

  static RobotConfig lookupConfig(RoboRio rio) {
    if (knownConfigs.containsKey(rio)) {
      return knownConfigs.get(rio);
    } else {
      return getCommonConfig();
    }
  }

  static RobotConfig getCommonConfig() {
    var common = new RobotConfig();
    common.hasSubsystems = false;
    common.hasProcessor = false;
    common.hasClimber = false;
    common.hasShooter = true;
    common.hasIntake = true;
    common.elevatorConfig = new Elevator.Config();
    common.shooterConfig = new Shooter.Config();
    common.intakeConfig = new Intake.Config();
    common.processorConfig = new Processor.Config();
    common.climberConfig = new Climber.Config();
    common.driverControllerPort = 0;
    common.operatorControllerPort = 1;
    common.elevatorConfig.hasFollowerMotor2 = false;
    common.shooterConfig.hasAlgaeMotor = false;

    return common;
  }

  static RobotConfig getV1Config() {
    var v1 = getCommonConfig();
    v1.hasClimber = false;
    v1.hasSubsystems = true;
    v1.hasIntake = false;
    v1.hasShooter = false;
    v1.climberConfig.climberMotorId = 3;
    v1.elevatorConfig.leaderMotorId = 4;
    v1.elevatorConfig.followerMotorId = 1;
    v1.elevatorConfig.lowerLimit = 0.1;
    v1.elevatorConfig.upperLimit = 75;
    v1.elevatorConfig.invertLeaderMotor = true;

    v1.elevatorConfig.kP_Up = .1;
    v1.elevatorConfig.kI_Up = 0;
    v1.elevatorConfig.kD_Up = 0;
    v1.elevatorConfig.kF_Up = 0;
    v1.elevatorConfig.maxVelocity_Up = 3000;
    v1.elevatorConfig.maxAcceleration_Up = 10000;
    v1.elevatorConfig.allowedError_Up = .2;

    v1.elevatorConfig.kP_Down = .1;
    v1.elevatorConfig.kI_Down = 0;
    v1.elevatorConfig.kD_Down = 0;
    v1.elevatorConfig.kF_Down = 0;
    v1.elevatorConfig.maxVelocity_Down = 300;
    v1.elevatorConfig.maxAcceleration_Down = 1000;
    v1.elevatorConfig.allowedError_Down = .2;

    v1.shooterConfig.shooterMotorId = 20;
    v1.intakeConfig.intakeMotorId = 5;
    v1.processorConfig.processorMotorId = 51; // fix me
    v1.processorConfig.deployProcessorMotorId = 52; // fix me
    v1.processorConfig.processorEncoderId = 53; // fix me
    v1.processorConfig.kP = 0.001;
    v1.processorConfig.kI = 0;
    v1.processorConfig.kD = 0;
    v1.processorConfig.kF = 0;
    v1.processorConfig.stowedPosition = 0;
    v1.processorConfig.deployedPosition = .3;
    v1.name = "V1";

    v1.drivetrainConfig = new Drivetrain.Config();
    v1.drivetrainConfig.frontLeftModule = Drivetrain.Config.Module.F;
    v1.drivetrainConfig.frontRightModule = Drivetrain.Config.Module.J;
    v1.drivetrainConfig.backLeftModule = Drivetrain.Config.Module.H;
    v1.drivetrainConfig.backRightModule = Drivetrain.Config.Module.G;
    v1.drivetrainConfig.sidelength = Inches.of(24);

    return v1;
  }

  static RobotConfig getProtoBoardConfig() {
    var protoBoard = getCommonConfig();
    protoBoard.hasIntake = false;
    protoBoard.hasShooter = false;
    protoBoard.elevatorConfig.leaderMotorId = 1;
    protoBoard.elevatorConfig.followerMotorId = 3;
    protoBoard.elevatorConfig.lowerLimit = 0.1;
    protoBoard.elevatorConfig.upperLimit = -5.1;
    protoBoard.elevatorConfig.invertLeaderMotor = true;
    protoBoard.elevatorConfig.kP_Up = 6;
    protoBoard.shooterConfig.shooterMotorId = 20;
    protoBoard.intakeConfig.intakeMotorId = 23; // fix me
    protoBoard.processorConfig.processorMotorId = 27;
    protoBoard.processorConfig.deployProcessorMotorId = 22;
    protoBoard.processorConfig.processorEncoderId = 22;
    protoBoard.processorConfig.kP = 0.3;
    protoBoard.processorConfig.kI = 0;
    protoBoard.processorConfig.kD = 0;
    protoBoard.processorConfig.kF = 0;
    protoBoard.processorConfig.stowedPosition = 0.25;
    protoBoard.processorConfig.deployedPosition = 0.55;
    protoBoard.name = "ProtoBoard";

    return protoBoard;
  }

  static RobotConfig getV2Config() {
    var v2 = getCommonConfig();
    v2.name = "V2";

    v2.drivetrainConfig = new Drivetrain.Config();
    v2.drivetrainConfig.frontLeftModule = Drivetrain.Config.Module.W;
    v2.drivetrainConfig.frontRightModule = Drivetrain.Config.Module.I;
    v2.drivetrainConfig.backLeftModule = Drivetrain.Config.Module.E;
    v2.drivetrainConfig.backRightModule = Drivetrain.Config.Module.K;
    v2.drivetrainConfig.sidelength = Inches.of(26);
    v2.drivetrainConfig.servoSpeed = 0.1;

    v2.hasSubsystems = false;
    v2.elevatorConfig.leaderMotorId = 3;
    v2.elevatorConfig.followerMotorId = 2;
    v2.elevatorConfig.lowerLimit = .5;
    v2.elevatorConfig.upperLimit = 79;
    v2.elevatorConfig.invertLeaderMotor = true;

    v2.elevatorConfig.kP_Up = 0.15;
    v2.elevatorConfig.kI_Up = 0;
    v2.elevatorConfig.kD_Up = 0.03;
    v2.elevatorConfig.kF_Up = 0.0002;
    v2.elevatorConfig.maxVelocity_Up = 5000;
    v2.elevatorConfig.maxAcceleration_Up = 10000;
    v2.elevatorConfig.allowedError_Up = .1;

    v2.elevatorConfig.kP_Down = .12;
    v2.elevatorConfig.kI_Down = 0;
    v2.elevatorConfig.kD_Down = 0.01;
    v2.elevatorConfig.kF_Down = 0.0002;
    v2.elevatorConfig.maxVelocity_Down = 5000;
    v2.elevatorConfig.maxAcceleration_Down = 10000;
    v2.elevatorConfig.allowedError_Down = .1;

    v2.hasIntake = false;
    v2.hasShooter = false;
    v2.shooterConfig.shooterMotorId = 12;
    v2.intakeConfig.intakeMotorId = 25;

    v2.hasClimber = false;
    v2.climberConfig.climberMotorId = 21;
    v2.climberConfig.invertMotors = true;
    v2.climberConfig.climberReverseSoftLimit = 0.045;
    v2.climberConfig.climberDeploySoftLimit = 0.405;

    v2.visionConfig = new Vision.Config();
    v2.visionConfig.frontCameraToRobot = new Transform3d(
        new Translation3d(Inches.of(6.809), Inches.of(-7.250), Inches.of(38.252)),
        new Rotation3d(Degrees.of(25), Degrees.of(0), Degrees.of(0)));
    v2.visionConfig.backRightCameraToRobot = new Transform3d(
        new Translation3d(Inches.of(-7.97), Inches.of(2.265), Inches.of(9.424)),
        new Rotation3d(Degrees.of(13.5), Degrees.of(0), Degrees.of(165)));

    return v2;
  }

  static RobotConfig getV3Config() {
    var v3 = getCommonConfig();
    v3.name = "V3";

    v3.hasIntake = true;
    v3.hasShooter = true;

    v3.drivetrainConfig = new Drivetrain.Config();
    v3.drivetrainConfig.frontLeftModule = Drivetrain.Config.Module.B;
    v3.drivetrainConfig.frontRightModule = Drivetrain.Config.Module.A;
    v3.drivetrainConfig.backLeftModule = Drivetrain.Config.Module.G;
    v3.drivetrainConfig.backRightModule = Drivetrain.Config.Module.C;
    v3.drivetrainConfig.sidelength = Inches.of(26);
    v3.drivetrainConfig.shouldUsePIDForAlignment = false;
    v3.hasSubsystems = true;
    v3.elevatorConfig.hasFollowerMotor2 = true;
    v3.elevatorConfig.leaderMotorId = 2;
    v3.elevatorConfig.followerMotorId = 1;
    v3.elevatorConfig.followerMotor2Id = 4;
    v3.elevatorConfig.lowerLimit = .5;
    v3.elevatorConfig.upperLimit = 81;
    v3.elevatorConfig.invertLeaderMotor = false;

    v3.elevatorConfig.kP_Up = 0.16;
    v3.elevatorConfig.kI_Up = 0.0;
    v3.elevatorConfig.kD_Up = 0.0;
    v3.elevatorConfig.kF_Up = 0.25;
    v3.elevatorConfig.maxVelocity_Up = 5000;
    v3.elevatorConfig.maxAcceleration_Up = 10000;
    v3.elevatorConfig.allowedError_Up = .2;

    v3.elevatorConfig.kP_Down = 0.16;
    v3.elevatorConfig.kI_Down = 0.0;
    v3.elevatorConfig.kD_Down = 0.0;
    v3.elevatorConfig.kF_Down = 0.25;
    v3.elevatorConfig.maxVelocity_Down = 5000;
    v3.elevatorConfig.maxAcceleration_Down = 10000;
    v3.elevatorConfig.allowedError_Down = .2;

    v3.shooterConfig.shooterMotorId = 12;
    v3.shooterConfig.algaeMotorId = 61;
    v3.shooterConfig.hasAlgaeMotor = true;
    v3.shooterConfig.algaeKP = 3.25;
    v3.shooterConfig.algaeKI = 0.000200;
    v3.shooterConfig.algaeKD = 0;
    v3.shooterConfig.algaeKF = 0;
    v3.intakeConfig.intakeMotorId = 5;
    v3.intakeConfig.servoClosePosition = 60;
    v3.intakeConfig.servoOpenPosition = 130;

    v3.hasClimber = true;
    v3.climberConfig.climberMotorId = 21;
    v3.climberConfig.invertMotors = true;
    v3.climberConfig.climberReverseSoftLimit = 0.058;
    v3.climberConfig.climberDeploySoftLimit = 0.422;
    v3.visionConfig = new Vision.Config();

    // v3.visionConfig.frontCameraToRobot = new Transform3d(
    //     new Translation3d(Inches.of(6.996), Inches.of(7.250), Inches.of(38.075)),
    //     new Rotation3d(Degrees.of(0), Degrees.of(-30), Degrees.of(0)));

    v3.visionConfig.backLeftCameraToRobot = new Transform3d(
        new Translation3d(Inches.of(-9.3), Inches.of(-3.80), Inches.of(9.51)),
        new Rotation3d(
            Degrees.of(0.41 + 0.3 + 0.2 + 0.05),
            Degrees.of(-14.69 - 0.75 - 0.3 - 0.1),
            Degrees.of(150.24)));

    v3.visionConfig.backRightCameraToRobot = new Transform3d(
        new Translation3d(Inches.of(-7.7), Inches.of(3.5), Inches.of(9.51)),
        new Rotation3d(
            Degrees.of(-0.3 - 1.77 - 0.4 + 0.3),
            Degrees.of(-17.5 - 1.78 + 0.8),
            Degrees.of(215.1)));

    return v3;
  }
}
