package frc.robot.telemetry.types;

import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class IntegerTelemetryEntry extends TelemetryEntry {
    private final IntegerLogEntry logEntry;
    private int lastValue;

    public IntegerTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public IntegerTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(path, shouldNT, shouldLazyLog);

        logEntry = new IntegerLogEntry(DataLogManager.getLog(), path);
    }

    public void append(int value) {
        if (shouldLog(lastValue == value)) {
            logEntry.append(value);

            if (networkEntry != null) {
                networkEntry.setNumber(value);
            }
            lastValue = value;
        }
    }
}
