package frc.robot.utils.logging;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;

public class PowerDistributionLogger {
    private final static String tableName = "/power/";
    private static int instances = 0;
    private final PowerDistribution powerDistribution;

    private final DoubleLogEntry totalEnergyEntry;
    private final DoubleLogEntry totalPowerEntry;
    private final DoubleLogEntry totalCurrentEntry;
    private final DoubleLogEntry temperatureEntry;
    private final DoubleLogEntry inputVoltageEntry;

    public PowerDistributionLogger(PowerDistribution powerDistribution) {
        instances++;
        this.powerDistribution = powerDistribution;

        String thisTableName = tableName + instances + "/";
        totalEnergyEntry = new DoubleLogEntry(DataLogManager.getLog(), thisTableName + "totalEnergy");
        totalPowerEntry = new DoubleLogEntry(DataLogManager.getLog(), thisTableName + "totalPower");
        totalCurrentEntry = new DoubleLogEntry(DataLogManager.getLog(), thisTableName + "totalCurrent");
        temperatureEntry = new DoubleLogEntry(DataLogManager.getLog(), thisTableName + "temperature");
        inputVoltageEntry = new DoubleLogEntry(DataLogManager.getLog(), thisTableName + "inputVoltage");
    }

    public void logValues() {
        totalEnergyEntry.append(powerDistribution.getTotalEnergy());
        totalPowerEntry.append(powerDistribution.getTotalPower());
        totalCurrentEntry.append(powerDistribution.getTotalCurrent());
        temperatureEntry.append(powerDistribution.getTemperature());
        inputVoltageEntry.append(powerDistribution.getVoltage());
    }
}
