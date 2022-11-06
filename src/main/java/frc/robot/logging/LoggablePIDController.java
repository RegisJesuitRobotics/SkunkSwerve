package frc.robot.logging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class LoggablePIDController extends PIDController {
    private final DoubleLogEntry currentMeasurementLogEntry;
    private final DoubleLogEntry setpointLogEntry;
    private final DoubleLogEntry outputLogEntry;

    private final DoublePublisher currentMeasurementNetworkEntry;
    private final DoublePublisher setpointNetworkEntry;
    private final DoublePublisher outputNetworkEntry;

    public LoggablePIDController(String logTable, NetworkTable networkTablesTable, double kp, double ki, double kd) {
        this(logTable, networkTablesTable, kp, ki, kd, 0.02);
    }

    public LoggablePIDController(
            String logTable, NetworkTable networkTablesTable, double kp, double ki, double kd, double period
    ) {
        super(kp, ki, kd, period);

        logTable += "/";
        DataLog dataLog = DataLogManager.getLog();
        currentMeasurementLogEntry = new DoubleLogEntry(dataLog, logTable + "currentMeasurement");
        setpointLogEntry = new DoubleLogEntry(dataLog, logTable + "setpoint");
        outputLogEntry = new DoubleLogEntry(dataLog, logTable + "output");

        currentMeasurementNetworkEntry = networkTablesTable.getDoubleTopic("currentMeasurement").publish();
        setpointNetworkEntry = networkTablesTable.getDoubleTopic("setpoint").publish();
        outputNetworkEntry = networkTablesTable.getDoubleTopic("output").publish();
    }

    @Override
    public void setSetpoint(double setpoint) {
        setpointLogEntry.append(setpoint);
        setpointNetworkEntry.set(setpoint);
        super.setSetpoint(setpoint);
    }

    @Override
    public double calculate(double measurement) {
        currentMeasurementLogEntry.append(measurement);
        currentMeasurementNetworkEntry.set(measurement);

        double output = super.calculate(measurement);
        outputLogEntry.append(output);
        outputNetworkEntry.set(output);

        return output;
    }
}
