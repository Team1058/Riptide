package frc.robot;

import java.util.HashMap;
import java.util.Map;

public class RoboRio {

  private static Map<String, RoboRio> map_by_name = new HashMap<String, RoboRio>();
  private static Map<String, RoboRio> map_by_serial_number = new HashMap<String, RoboRio>();

  final String name;
  final String serial_number;

  RoboRio(String name, String serial_number) {
    this.name = name;
    this.serial_number = serial_number;
    map_by_name.put(name, this);
    map_by_serial_number.put(serial_number, this);
  }

  public boolean match(String rio_serial_number) {
    return this.serial_number.equals(rio_serial_number);
  }

  @Override
  public String toString() {
    return name;
  }

  public static RoboRio lookupByName(String name) {
    return map_by_name.get(name);
  }

  public static RoboRio lookupBySerialNumber(String serial_number) {
    return map_by_serial_number.get(serial_number);
  }

  public static final RoboRio ALPHA = new RoboRio("ALPHA", "0313a676");
  public static final RoboRio BETA = new RoboRio("BETA", "0306ae25");
  public static final RoboRio GAMMA = new RoboRio("GAMMA", "0305ec4b");
  public static final RoboRio DELTA = new RoboRio("DELTA", "0312db68");
  public static final RoboRio EPSILON = new RoboRio("EPSILON", "023D2C29");
  public static final RoboRio ZETA = new RoboRio("ZETA", "23FF3A0");
}
