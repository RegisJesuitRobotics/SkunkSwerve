package frc.robot.utils;
// Modified from 6328 Mechanical Advantage

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.telemetry.types.DoubleTelemetryEntry;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns
 * default if not or value not in dashboard.
 */
public class TunableDouble {
    private final DoubleEntry networkEntry;
    private final DoubleTelemetryEntry telemetryEntry;
    private double lastHasChangedValue;

    /**
     * Create a new TunableNumber with the default value
     *
     * @param networkName  Name for network tables
     * @param defaultValue Default value
     */
    public TunableDouble(String networkName, double defaultValue) {
        this.networkEntry = NetworkTableInstance.getDefault().getDoubleTopic(networkName).getEntry(defaultValue);
        networkEntry.set(networkEntry.get());
        this.telemetryEntry = new DoubleTelemetryEntry(networkName, false);

        this.lastHasChangedValue = defaultValue;
    }

    /**
     * Get the current value from NT if available
     *
     * @return The current value
     */
    public double get() {
        double value = networkEntry.get();
        telemetryEntry.append(value);
        return value;
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @return True if the number has changed since the last time this method was
     *         called, false otherwise
     */
    public boolean hasChanged() {
        double currentValue = get();
        if (currentValue != lastHasChangedValue) {
            lastHasChangedValue = currentValue;
            System.out.println("IT CHANGED");
            return true;
        }

        return false;
    }
}
