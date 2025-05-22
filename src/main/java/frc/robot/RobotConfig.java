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
    knownConfigs.put(RoboRio.DELTA, getV3Config());
    knownConfigs.put(RoboRio.EPSILON, getV3Config());
  }

  static RobotConfig lookupConfig(RoboRio rio) {
    if (knownConfigs.containsKey(rio)) {
      return knownConfigs.get(rio);
    } else {
      return getV3Config();
    }
  }

  static RobotConfig getCommonConfig() {
    var common = new RobotConfig();
    common.hasSubsystems = false;
    common.hasClimber = false;
    common.hasShooter = true;
    common.elevatorConfig = new Elevator.Config();
    common.shooterConfig = new Shooter.Config();
    common.climberConfig = new Climber.Config();
    common.driverControllerPort = 0;
    common.operatorControllerPort = 1;
    common.elevatorConfig.hasFollowerMotor2 = false;
    common.shooterConfig.hasAlgaeMotor = false;

    return common;
  }

  static RobotConfig getV3Config() {
    var v3 = getCommonConfig();
    v3.name = "V3";

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
    v3.elevatorConfig.leaderMotorId = 1;
    v3.elevatorConfig.followerMotorId = 2;
    v3.elevatorConfig.followerMotor2Id = 3;
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

    v3.hasClimber = true;
    v3.climberConfig.climberLeaderMotorId = 4;
    v3.climberConfig.climberFollowerMotorId = 5;
    v3.climberConfig.invertLeaderMotor = true;
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
