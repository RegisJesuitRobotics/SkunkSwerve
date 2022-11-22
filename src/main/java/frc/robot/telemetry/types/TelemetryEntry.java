package frc.robot.telemetry.types;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class TelemetryEntry {
    protected final NetworkTableEntry networkEntry;
    protected final boolean shouldLazyLog;

    protected boolean firstRun = true;

    protected TelemetryEntry(String path, boolean shouldNT, boolean shouldLazyLog) {
        this.shouldLazyLog = shouldLazyLog;

        if (shouldNT) {
            networkEntry = NetworkTableInstance.getDefault().getEntry(path);
        } else {
            networkEntry = null;
        }
    }

    protected boolean shouldLog(boolean isLastEqual) {
        if (firstRun) {
            firstRun = false;
            return true;
        }
        return !isLastEqual;
    }
}
