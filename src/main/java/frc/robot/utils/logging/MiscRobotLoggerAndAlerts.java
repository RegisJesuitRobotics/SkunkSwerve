package frc.robot.utils.logging;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.MiscConstants;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;

public class MiscRobotLoggerAndAlerts {
    private static final String tableName = "/robot/";

    private final Alert lowBatteryVoltageAlert = new Alert("Low Battery Voltage", AlertType.WARNING);
    private final Alert highCanUsageAlert = new Alert("High CAN Usage", AlertType.WARNING);

    private final Alert[] controllerAlerts = new Alert[MiscConstants.usedControllerPorts.length];

    private final DoubleLogEntry voltageEntry = new DoubleLogEntry(DataLogManager.getLog(), tableName + "voltage");
    private final DoubleLogEntry canUtilizationEntry = new DoubleLogEntry(
            DataLogManager.getLog(), tableName + "canUse"
    );

    public MiscRobotLoggerAndAlerts() {
        for (int i = 0; i < controllerAlerts.length; i++) {
            controllerAlerts[i] = new Alert(
                    "Controller " + MiscConstants.usedControllerPorts[i] + " is disconnected.", AlertType.WARNING
            );
        }
    }

    public void logValues() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        voltageEntry.append(batteryVoltage);
        lowBatteryVoltageAlert.set(batteryVoltage < 11.0);

        double percentBusUsage = RobotController.getCANStatus().percentBusUtilization;
        canUtilizationEntry.append(percentBusUsage);
        highCanUsageAlert.set(percentBusUsage > 90.0);

        for (int i = 0; i < controllerAlerts.length; i++) {
            controllerAlerts[i].set(!DriverStation.isJoystickConnected(MiscConstants.usedControllerPorts[i]));
        }
    }
}
