package frc.robot.telemetry;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.telemetry.types.DoubleTelemetryEntry;

public class TelemetryPowerDistribution extends PowerDistribution {
    private final static String tableName = "/power/";

    private final DoubleTelemetryEntry totalEnergyEntry;
    private final DoubleTelemetryEntry totalPowerEntry;
    private final DoubleTelemetryEntry totalCurrentEntry;
    private final DoubleTelemetryEntry temperatureEntry;
    private final DoubleTelemetryEntry inputVoltageEntry;

    public TelemetryPowerDistribution(int module, ModuleType moduleType) {
        super(module, moduleType);
        super.resetTotalEnergy();

        String thisTableName = tableName + module + "/";
        totalEnergyEntry = new DoubleTelemetryEntry(thisTableName + "totalEnergy", true);
        totalPowerEntry = new DoubleTelemetryEntry(thisTableName + "totalPower", false);
        totalCurrentEntry = new DoubleTelemetryEntry(thisTableName + "totalCurrent", false);
        temperatureEntry = new DoubleTelemetryEntry(thisTableName + "temperature", false);
        inputVoltageEntry = new DoubleTelemetryEntry(thisTableName + "inputVoltage", false);
    }

    public void logValues() {
        totalEnergyEntry.append(super.getTotalEnergy());
        totalPowerEntry.append(super.getTotalPower());
        totalCurrentEntry.append(super.getTotalCurrent());
        temperatureEntry.append(super.getTemperature());
        inputVoltageEntry.append(super.getVoltage());
    }
}
