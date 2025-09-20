package pvc.runtime;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**  Execution Model
 *
 * RobotState is latched after each step in the process. Changes from one module are not visible to other modules until
 * the next phase. This ensures that things can't dependent on internal module order.
 *
 * 'preInput()' is called on all modules
 * 'readInputs()' is called on all subystem hardware implementations, or inputs are read from a replay log.
 * 'processInputs()' is called on all subsystems
 * 'postInput()' is called on all modules.
 * 'enable()' is called on all modules if we've transitioned since the last cycle.
 *  The command scheduler is run.
 * 'disable()' is called on all modules if we've transitioned since the last cycle.
 * 'preOutput()' is called on all modules
 * 'generateOutputs()' is called on all subsystems.
 * 'writeOutputs()' is called on all subsystem hardware implementations and outputs are written to the log.
 * 'postOutput()' is called on all modules.
 */
public class Robot extends LoggedRobot {

  private Runtime runtime;

  public Robot(RobotBuilder robotBuilder) {
    DriverStation.silenceJoystickConnectionWarning(true);
    /*
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("CommitHash", BuildConstants.GIT_SHA);
    Logger.recordMetadata("CommitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("Branch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("WorkingTree", "Dirty");
        // break;
      case 1:
        Logger.recordMetadata("WorkingTree", "Clean");
        break;
      default:
        Logger.recordMetadata("WorkingTree", "Unknown");
        break;
    }
    */

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath =
          LogFileUtil
              .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(
          LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    subsystems = new ArrayList<SubSystemDriver>();
  }

  private List<SubSystemDriver> subsystems;
  private CommandScheduler commandScheduler = CommandScheduler.getInstance();
  private RobotState robotState;

  @Override
  protected void loopFunc() {
    DriverStation.refreshData();

    subsystems.forEach((module) -> module.preInput(robotState));
    robotState.commit();

    subsystems.forEach((subsystem) -> subsystem.readAndProcessInputs(robotState));
    robotState.commit();

    subsystems.forEach((module) -> module.postInput(robotState));
    robotState.commit();

    commandScheduler.run();

    subsystems.forEach((module) -> module.preOutput(robotState));
    robotState.commit();

    subsystems.forEach((subsystem) -> subsystem.generateAndWriteOutputs(robotState));
    robotState.commit();
    robotState.toLog();

    subsystems.forEach((module) -> module.postOutput(robotState));
  }
}
