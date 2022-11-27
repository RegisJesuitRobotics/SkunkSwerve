package frc.robot.telemetry;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.telemetry.types.DoubleTelemetryEntry;

public class TelemetryPIDController extends PIDController {
    private final DoubleTelemetryEntry currentMeasurementEntry;
    private final DoubleTelemetryEntry setpointEntry;
    private final DoubleTelemetryEntry outputEntry;

    public TelemetryPIDController(String logTable, double kp, double ki, double kd) {
        this(logTable, kp, ki, kd, 0.02);
    }

    public TelemetryPIDController(String logTable, double kp, double ki, double kd, double period) {
        super(kp, ki, kd, period);

        logTable += "/";
        currentMeasurementEntry = new DoubleTelemetryEntry(logTable + "currentMeasurement", true);
        setpointEntry = new DoubleTelemetryEntry(logTable + "setpoint", true);
        outputEntry = new DoubleTelemetryEntry(logTable + "output", true);
    }

    @Override
    public void setSetpoint(double setpoint) {
        setpointEntry.append(setpoint);
        super.setSetpoint(setpoint);
    }

    @Override
    public double calculate(double measurement) {
        currentMeasurementEntry.append(measurement);

        double output = super.calculate(measurement);
        outputEntry.append(output);

        return output;
    }
}
