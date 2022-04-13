package frc.robot.utils.datalog;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

public class LazyDoubleLogEntry extends DoubleLogEntry {
    private boolean firstAppend = true;
    private double lastValue;

    public LazyDoubleLogEntry(DataLog log, String name, String metadata, long timestamp) {
        super(log, name, metadata, timestamp);
    }

    public LazyDoubleLogEntry(DataLog log, String name, String metadata) {
        super(log, name, metadata);
    }

    public LazyDoubleLogEntry(DataLog log, String name, long timestamp) {
        super(log, name, timestamp);
    }

    public LazyDoubleLogEntry(DataLog log, String name) {
        super(log, name);
    }

    @Override
    public void append(double value, long timestamp) {
        if (firstAppend || value != lastValue) {
            firstAppend = false;
            lastValue = value;
            super.append(value, timestamp);
        }
    }

    @Override
    public void append(double value) {
        append(value, 0);
    }
}
