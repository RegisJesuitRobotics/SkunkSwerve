package frc.robot.telemetry;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.telemetry.types.DoubleTelemetryEntry;

public class LoggablePIDController extends PIDController {
    private final DoubleTelemetryEntry currentMeasurementLogEntry;
    private final DoubleTelemetryEntry setpointLogEntry;
    private final DoubleTelemetryEntry outputLogEntry;

    public LoggablePIDController(String logTable, double kp, double ki, double kd) {
        this(logTable, kp, ki, kd, 0.02);
    }

    public LoggablePIDController(String logTable, double kp, double ki, double kd, double period) {
        super(kp, ki, kd, period);

        logTable += "/";
        currentMeasurementLogEntry = new DoubleTelemetryEntry(logTable + "currentMeasurement", true);
        setpointLogEntry = new DoubleTelemetryEntry(logTable + "setpoint", true);
        outputLogEntry = new DoubleTelemetryEntry(logTable + "output", true);
    }

    @Override
    public void setSetpoint(double setpoint) {
        setpointLogEntry.append(setpoint);
        super.setSetpoint(setpoint);
    }

    @Override
    public double calculate(double measurement) {
        currentMeasurementLogEntry.append(measurement);

        double output = super.calculate(measurement);
        outputLogEntry.append(output);

        return output;
    }
}
