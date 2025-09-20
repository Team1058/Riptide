package pvc.runtime;

import java.util.function.Consumer;
import java.util.function.Function;

public class RobotStateEntry<T> {

  public RobotStateEntry(String name, Function<Object, T> caster, Consumer<Object> logger) {
    this.name = name;
    this.caster = caster;
    this.logger = logger;
  }

  private final String name;
  private final Function<Object, T> caster;
  private final Consumer<Object> logger;

  String name()
  {
    return name;
  }

  T cast(Object value)
  {
    return caster.apply(value);
  }

  void toLog(Object value)
  {
    logger.accept(value);
  }
}
