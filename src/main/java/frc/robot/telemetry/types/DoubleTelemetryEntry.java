package frc.robot.telemetry.types;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class DoubleTelemetryEntry extends TelemetryEntry {
    private final DoubleLogEntry logEntry;
    private double lastValue;

    public DoubleTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public DoubleTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(path, shouldNT, shouldLazyLog);

        logEntry = new DoubleLogEntry(DataLogManager.getLog(), path);
    }

    public void append(double value) {
        if (shouldLog(lastValue == value)) {
            logEntry.append(value);

            if (networkEntry != null) {
                networkEntry.setDouble(value);
            }
            lastValue = value;
        }
    }
}
