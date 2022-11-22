package frc.robot.telemetry.types;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class StringTelemetryEntry extends TelemetryEntry {
    private final StringLogEntry logEntry;
    private String lastValue;

    public StringTelemetryEntry(String path, boolean shouldNT) {
        this(path, shouldNT, true);
    }

    public StringTelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        super(path, shouldNT, shouldLazyLog);

        logEntry = new StringLogEntry(DataLogManager.getLog(), path);
    }

    public void append(String value) {
        if (shouldLog(lastValue.equals(value))) {
            logEntry.append(value);

            if (networkEntry != null) {
                networkEntry.setString(value);
            }
            lastValue = value;
        }
    }
}
