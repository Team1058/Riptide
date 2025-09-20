package pvc.runtime;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

public class RobotState {
  Map<RobotStateEntry<?>, Object> active_state = new HashMap<RobotStateEntry<?>, Object>();
  Map<RobotStateEntry<?>, Object> pending_updates = new HashMap<RobotStateEntry<?>, Object>();

  private static String PREFIX = "RobotState/";

  public static RobotStateEntry<Integer> make_entry(String name, Integer default_value)
  {
    return new RobotStateEntry<Integer>(
      name,
      (value) -> (Integer)value,
      (value) -> Logger.recordOutput(PREFIX + name, (Integer)value));
  }

  public static RobotStateEntry<Double> make_entry(String name, Double default_value)
  {
    return new RobotStateEntry<Double>(
      name,
      (value) -> (Double)value,
      (value) -> Logger.recordOutput(PREFIX + name, (Double)value));
  }

  public <T> void set(RobotStateEntry<T> entry, T value)
  {
    pending_updates.put(entry, value);
  }

  public <T> T get(RobotStateEntry<T> entry)
  {
    return entry.cast(active_state.get(entry));
  }

  void commit()
  {
    active_state.putAll(pending_updates);
    active_state.clear();
  }

  void toLog()
  {
    active_state.forEach((entry, value) -> entry.toLog(value));
  }

}
