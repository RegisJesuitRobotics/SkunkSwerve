package frc.robot.logging;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

public class LoggableTalonFX extends TalonFX {
    private final DoubleLogEntry outputAmpsEntry;
    private final DoubleLogEntry inputAmpsEntry;
    private final DoubleLogEntry outputVoltageEntry;
    private final DoubleLogEntry busVoltageEntry;
    private final DoubleLogEntry temperatureEntry;
    private final BooleanLogEntry inBrakeModeEntry;

    private double lastLogTime = 0.0;

    public LoggableTalonFX(int deviceNumber, String logTable, String canbus) {
        super(deviceNumber, canbus);

        logTable += "/";
        DataLog log = DataLogManager.getLog();
        outputAmpsEntry = new DoubleLogEntry(log, logTable + "outputAmps");
        inputAmpsEntry = new DoubleLogEntry(log, logTable + "inputAmps");
        outputVoltageEntry = new DoubleLogEntry(log, logTable + "outputVoltage");
        busVoltageEntry = new DoubleLogEntry(log, logTable + "busVoltage");
        temperatureEntry = new DoubleLogEntry(log, logTable + "temperature");
        inBrakeModeEntry = new BooleanLogEntry(log, logTable + "inBrakeMode");

        IntegerLogEntry firmwareVersionEntry = new IntegerLogEntry(log, logTable + "firmwareVersion");
        firmwareVersionEntry.append(super.getFirmwareVersion());
        firmwareVersionEntry.finish();
    }

    public LoggableTalonFX(int deviceNumber, String logTable) {
        this(deviceNumber, logTable, "");
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        inBrakeModeEntry.append(neutralMode == NeutralMode.Brake);
        super.setNeutralMode(neutralMode);
    }

    public void logValues() {
        if (Timer.getFPGATimestamp() - lastLogTime > 1.0) {
            lastLogTime = Timer.getFPGATimestamp();
            outputAmpsEntry.append(super.getStatorCurrent());
            inputAmpsEntry.append(super.getSupplyCurrent());
            outputVoltageEntry.append(super.getMotorOutputPercent());
            busVoltageEntry.append(super.getBusVoltage());
            temperatureEntry.append(super.getTemperature());
        }
    }
}
