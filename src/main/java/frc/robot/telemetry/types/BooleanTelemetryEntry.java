package frc.robot.telemetry.types;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class BooleanTelemetryEntry extends TelemetryEntry {
    private final BooleanLogEntry logEntry;
    private boolean lastValue;

    public BooleanTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public BooleanTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(path, shouldNT, shouldLazyLog);

        logEntry = new BooleanLogEntry(DataLogManager.getLog(), path);
    }

    public void append(boolean value) {
        if (shouldLog(lastValue == value)) {
            logEntry.append(value);

            if (networkEntry != null) {
                networkEntry.setBoolean(value);
            }
            lastValue = value;
        }
    }
}
