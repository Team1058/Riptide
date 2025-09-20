package pvc.runtime;

public class RobotBuilder {

  public static class BuilderContext {}

  public RobotBuilder() {}

  public void buildRealRobot(BuilderContext ctx) {
    throw new UnsupportedOperationException("No real robots available.");
  }

  public void buildReplayRobot(BuilderContext ctx, RobotConfig loggedConfig) {
    throw new UnsupportedOperationException("No replay robot available.");
  }

  public void buildSimRobot(BuilderContext ctx, SimState simState) {
    throw new UnsupportedOperationException("No sim robot available.");
  }
}
