package frc.robot.utils.logging;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;

public class MiscRobotLogger {
    private static final String tableName = "/robot/";
    private static final DoubleLogEntry voltageEntry = new DoubleLogEntry(
            DataLogManager.getLog(), tableName + "voltage"
    );
    private static final DoubleLogEntry canUtilizationEntry = new DoubleLogEntry(
            DataLogManager.getLog(), tableName + "canUse"
    );

    private static final BooleanLogEntry isBrownedOutEntry = new BooleanLogEntry(
            DataLogManager.getLog(), tableName + "isBrownedOut"
    );

    public static void logValues() {
        voltageEntry.append(RobotController.getBatteryVoltage());
        canUtilizationEntry.append(RobotController.getCANStatus().percentBusUtilization);
        isBrownedOutEntry.append(RobotController.isBrownedOut());
    }
}
