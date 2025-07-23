    // Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableConstant implements DoubleSupplier {
    private static final String tableKey = "TunableNumbers";
    private boolean tuningMode = false;
    private final String key;
    private boolean hasDefault = false;
    private double defaultValue;
    private LoggedNetworkNumber dashboardNumber;
    private double lastValue;

    /**
     * Create a new LoggedTunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public TunableConstant(String dashboardKey)
    {
        this.key = tableKey + "/" + dashboardKey;
    }

    /**
     * Create a new LoggedTunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public TunableConstant(String dashboardKey, double defaultValue)
    {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    public void setTuningMode(boolean enable)
    {
        tuningMode = enable;
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    public void initDefault(double defaultValue)
    {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            if (tuningMode) {
                dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public double get()
    {
        if (!hasDefault) {
            return 0.0;
        } else {
            return tuningMode ? dashboardNumber.get() : defaultValue;
        }
    }
    
    private double getAndUpdate(){
        lastValue = get();
        return lastValue;
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *        objects. Recommended approach is to pass the result of "hashCode()"
     * @return True if the number has changed since the last time this method was called, false
     *         otherwise.
     */
    public boolean hasChanged()
    {
        double currentValue = get();
        return currentValue != lastValue;
    }

    /**
     * Runs action if any of the tunableNumbers have changed
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
     *        objects. Recommended approach is to pass the result of "hashCode()"
     * @param action Callback to run when any of the tunable numbers have changed. Access tunable
     *        numbers in order inputted in method
     * @param tunableNumbers All tunable numbers to check
     */
    public static void ifChanged(
        Consumer<double[]> action, TunableConstant... tunableNumbers)
    {
        if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged())) {
            action.accept(
                Arrays.stream(tunableNumbers).mapToDouble(TunableConstant::getAndUpdate).toArray());
        }
    }

    /** Runs action if any of the tunableNumbers have changed */
    public static void ifChanged(Runnable action, TunableConstant... tunableNumbers)
    {
        ifChanged(values -> action.run(), tunableNumbers);
    }

    @Override
    public double getAsDouble()
    {
        return get();
    }

}
