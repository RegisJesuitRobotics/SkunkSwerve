package frc.robot.utils;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;

public class MiscRobotLogger {
    private static final String tableName = "/robot/";
    private static final DoubleLogEntry voltageEntry = new DoubleLogEntry(
            DataLogManager.getLog(), tableName + "voltage"
    );
    private static final DoubleLogEntry canUtilization = new DoubleLogEntry(
            DataLogManager.getLog(), tableName + "canUse"
    );

    public static void logValues() {
        voltageEntry.append(RobotController.getBatteryVoltage());
        canUtilization.append(RobotController.getCANStatus().percentBusUtilization);
    }
}
