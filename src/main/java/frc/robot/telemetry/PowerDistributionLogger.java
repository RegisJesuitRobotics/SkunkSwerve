package frc.robot.telemetry;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.telemetry.types.DoubleTelemetryEntry;

public class PowerDistributionLogger {
    private final static String tableName = "/power/";
    private static int instances = 0;
    private final PowerDistribution powerDistribution;

    private final DoubleTelemetryEntry totalEnergyEntry;
    private final DoubleTelemetryEntry totalPowerEntry;
    private final DoubleTelemetryEntry totalCurrentEntry;
    private final DoubleTelemetryEntry temperatureEntry;
    private final DoubleTelemetryEntry inputVoltageEntry;

    public PowerDistributionLogger(PowerDistribution powerDistribution) {
        instances++;
        this.powerDistribution = powerDistribution;
        powerDistribution.resetTotalEnergy();

        String thisTableName = tableName + instances + "/";
        totalEnergyEntry = new DoubleTelemetryEntry(thisTableName + "totalEnergy", true);
        totalPowerEntry = new DoubleTelemetryEntry(thisTableName + "totalPower", false);
        totalCurrentEntry = new DoubleTelemetryEntry(thisTableName + "totalCurrent", false);
        temperatureEntry = new DoubleTelemetryEntry(thisTableName + "temperature", false);
        inputVoltageEntry = new DoubleTelemetryEntry(thisTableName + "inputVoltage", false);
    }

    public void logValues() {
        totalEnergyEntry.append(powerDistribution.getTotalEnergy());
        totalPowerEntry.append(powerDistribution.getTotalPower());
        totalCurrentEntry.append(powerDistribution.getTotalCurrent());
        temperatureEntry.append(powerDistribution.getTemperature());
        inputVoltageEntry.append(powerDistribution.getVoltage());
    }
}
