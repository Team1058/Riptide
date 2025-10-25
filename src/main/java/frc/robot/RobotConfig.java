package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import java.util.HashMap;
import java.util.Map;

public class RobotConfig {
  String name;
  int driverControllerPort;
  int operatorControllerPort;
  boolean hasSubsystems;
  boolean hasClimber;
  boolean hasShooter;
  Elevator.Config elevatorConfig;
  Shooter.Config shooterConfig;
  Climber.Config climberConfig;
  Drivetrain.Config drivetrainConfig;
  Vision.Config visionConfig;

  static Map<RoboRio, RobotConfig> knownConfigs;

  static {
    knownConfigs = new HashMap<RoboRio, RobotConfig>();
    knownConfigs.put(RoboRio.DELTA, getriptideConfig());
    knownConfigs.put(RoboRio.EPSILON, getriptideConfig());
  }

  static RobotConfig lookupConfig(RoboRio rio) {
    if (knownConfigs.containsKey(rio)) {
      return knownConfigs.get(rio);
    } else {
      return getriptideConfig();
    }
  }

  static RobotConfig getCommonConfig() {
    var common = new RobotConfig();
    common.hasSubsystems = false;
    common.hasClimber = false;
    common.hasShooter = false;
    common.elevatorConfig = new Elevator.Config();
    common.shooterConfig = new Shooter.Config();
    common.climberConfig = new Climber.Config();
    common.driverControllerPort = 0;
    common.operatorControllerPort = 1;
    common.elevatorConfig.hasFollowerMotor2 = false;
    common.shooterConfig.hasAlgaeMotor = false;

    return common;
  }

  static RobotConfig getriptideConfig() {
    var riptide = getCommonConfig();
    riptide.name = "riptide";

    riptide.hasShooter = true;

    riptide.drivetrainConfig = new Drivetrain.Config();
    riptide.drivetrainConfig.frontLeftModule = Drivetrain.Config.Module.E;
    riptide.drivetrainConfig.frontRightModule = Drivetrain.Config.Module.W;
    riptide.drivetrainConfig.backLeftModule = Drivetrain.Config.Module.K;
    riptide.drivetrainConfig.backRightModule = Drivetrain.Config.Module.D;
    riptide.drivetrainConfig.sidelength = Inches.of(26);
    riptide.drivetrainConfig.shouldUsePIDForAlignment = false;
    riptide.hasSubsystems = true;
    riptide.elevatorConfig.hasFollowerMotor2 = true;
    riptide.elevatorConfig.leaderMotorId = 1;
    riptide.elevatorConfig.followerMotorId = 2;
    riptide.elevatorConfig.followerMotor2Id = 3;
    riptide.elevatorConfig.lowerLimit = .3;
    riptide.elevatorConfig.upperLimit = 31.2;
    riptide.elevatorConfig.invertLeaderMotor = true;

    // Check this?
    // riptide.elevatorConfig.kP_Up = 0.03;
    riptide.elevatorConfig.kP_Up = 0.075;
    riptide.elevatorConfig.kI_Up = 0.0;
    riptide.elevatorConfig.kD_Up = 0.0;
    riptide.elevatorConfig.kF_Up = 0.5;
    riptide.elevatorConfig.maxVelocity_Up = 5000;
    riptide.elevatorConfig.maxAcceleration_Up = 10000;
    riptide.elevatorConfig.allowedError_Up = .075;

    riptide.elevatorConfig.kP_Down = 0.15;
    // riptide.elevatorConfig.kP_Down = 0.8;
    riptide.elevatorConfig.kI_Down = 0.0;
    // riptide.elevatorConfig.kD_Down = 0.00;
    riptide.elevatorConfig.kD_Down = 0.003;
    riptide.elevatorConfig.kF_Down = 0.5;
    riptide.elevatorConfig.maxVelocity_Down = 5000;
    riptide.elevatorConfig.maxAcceleration_Down = 10000;
    riptide.elevatorConfig.allowedError_Down = .075;

    riptide.shooterConfig.shooterMotorId = 6;
    riptide.shooterConfig.algaeMotorId = 25;
    riptide.shooterConfig.intakeFlapMotorID = 12;
    riptide.shooterConfig.hasAlgaeMotor = true;
    riptide.shooterConfig.algaeKP = 3.25 / 10;
    riptide.shooterConfig.algaeKI = 0.0002 / 10;
    riptide.shooterConfig.algaeKD = 0;
    riptide.shooterConfig.algaeKF = 0;
    riptide.shooterConfig.flapForwardLimit = 0;
    riptide.shooterConfig.flapReversedLimit = -4;

    riptide.hasClimber = true;
    riptide.climberConfig.climberLeaderMotorId = 4;
    riptide.climberConfig.climberFollowerMotorId = 5;
    riptide.climberConfig.invertLeaderMotor = false;
    // riptide.climberConfig.climberClimbed = 0.8-0.1;
    riptide.climberConfig.climberClimbed = 0.64;
    riptide.climberConfig.climberDeployed = 0.38 - 0.1;
    riptide.visionConfig = new Vision.Config();

    // riptide.visionConfig.frontCameraToRobot = new Transform3d(
    //     new Translation3d(Inches.of(6.996), Inches.of(7.250), Inches.of(38.075)),
    //     new Rotation3d(Degrees.of(0), Degrees.of(-30), Degrees.of(0)));

    // These are (theoretically) close to correct
    riptide.visionConfig.rightCameraToRobot = new Transform3d(
        new Translation3d(Inches.of(11.186), Inches.of(-9.091), Inches.of(9.026)),
        // new Translation3d(Inches.of(0), Inches.of(0), Inches.of(0)),
        new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(28 + 1)));

    riptide.visionConfig.leftCameraToRobot = new Transform3d(
        new Translation3d(Inches.of(11.186), Inches.of(9.091), Inches.of(9.026)),
        new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(-28)));

    return riptide;
  }
}
