package frc.robot.logging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class LoggablePIDController extends PIDController {
    private final DoubleLogEntry currentMeasurementLogEntry;
    private final DoubleLogEntry setpointLogEntry;
    private final DoubleLogEntry outputLogEntry;

    private final NetworkTableEntry currentMeasurementNetworkEntry;
    private final NetworkTableEntry setpointNetworkEntry;
    private final NetworkTableEntry outputNetworkEntry;

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

        currentMeasurementNetworkEntry = networkTablesTable.getEntry("currentMeasurement");
        setpointNetworkEntry = networkTablesTable.getEntry("setpoint");
        outputNetworkEntry = networkTablesTable.getEntry("output");
    }

    @Override
    public void setSetpoint(double setpoint) {
        setpointLogEntry.append(setpoint);
        setpointNetworkEntry.setDouble(setpoint);
        super.setSetpoint(setpoint);
    }

    @Override
    public double calculate(double measurement) {
        currentMeasurementLogEntry.append(measurement);
        currentMeasurementNetworkEntry.setDouble(measurement);

        double output = super.calculate(measurement);
        outputLogEntry.append(output);
        outputNetworkEntry.setDouble(output);

        return output;
    }
}
