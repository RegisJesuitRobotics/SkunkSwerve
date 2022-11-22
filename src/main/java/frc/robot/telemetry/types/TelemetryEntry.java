package frc.robot.telemetry.types;

public abstract class TelemetryEntry {
    protected final boolean shouldLazyLog;

    protected boolean firstRun = true;

    protected TelemetryEntry(boolean shouldLazyLog) {
        this.shouldLazyLog = shouldLazyLog;
    }

    protected boolean shouldLog(boolean isLastEqual) {
        if (firstRun) {
            firstRun = false;
            return true;
        }
        return !isLastEqual && shouldLazyLog;
    }
}
