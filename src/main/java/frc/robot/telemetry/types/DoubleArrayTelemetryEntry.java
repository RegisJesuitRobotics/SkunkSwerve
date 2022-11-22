package frc.robot.telemetry.types;

import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.Arrays;

public class DoubleArrayTelemetryEntry extends TelemetryEntry {
    private final DoubleArrayLogEntry logEntry;
    private double[] lastValue;

    public DoubleArrayTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public DoubleArrayTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(path, shouldNT, shouldLazyLog);

        logEntry = new DoubleArrayLogEntry(DataLogManager.getLog(), path);
    }

    public void append(double[] value) {
        if (shouldLog(Arrays.equals(lastValue, value))) {
            logEntry.append(value);

            if (networkEntry != null) {
                networkEntry.setDoubleArray(value);
            }
            lastValue = value;
        }
    }
}
