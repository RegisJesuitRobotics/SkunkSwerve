package frc.robot.telemetry.types.rich;

import edu.wpi.first.hal.can.CANStatus;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;

public class CANBusDataEntry {
    private final DoubleTelemetryEntry percentUtilizationEntry;
    private final IntegerTelemetryEntry offCountEntry;
    private final IntegerTelemetryEntry txErrorCountEntry;
    private final IntegerTelemetryEntry rxErrorCountEntry;
    private final IntegerTelemetryEntry transmitFullCountEntry;

    public CANBusDataEntry(String path) {
        path += "/";
        percentUtilizationEntry = new DoubleTelemetryEntry(path + "percentUtilization", false);
        offCountEntry = new IntegerTelemetryEntry(path + "offCount", false);
        txErrorCountEntry = new IntegerTelemetryEntry(path + "transmitErrorCount", false);
        rxErrorCountEntry = new IntegerTelemetryEntry(path + "receiveErrorCount", false);
        transmitFullCountEntry = new IntegerTelemetryEntry(path + "transmitFullCount", false);
    }

    public void append(CANStatus data) {
        percentUtilizationEntry.append(data.percentBusUtilization);
        offCountEntry.append(data.busOffCount);
        txErrorCountEntry.append(data.transmitErrorCount);
        rxErrorCountEntry.append(data.receiveErrorCount);
        transmitFullCountEntry.append(data.txFullCount);
    }
}
