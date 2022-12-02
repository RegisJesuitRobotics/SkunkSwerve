package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import frc.robot.telemetry.TunableTelemetryPIDController;

public class TunablePIDGains {
    public final TunableDouble p;
    public final TunableDouble i;
    public final TunableDouble d;

    public TunablePIDGains(String networkName, double p, double i, double d) {
        networkName += "/";
        this.p = new TunableDouble(networkName + "p", p);
        this.i = new TunableDouble(networkName + "i", i);
        this.d = new TunableDouble(networkName + "d", d);
    }

    public void setSlot(SlotConfiguration slot) {
        slot.kP = p.get();
        slot.kI = i.get();
        slot.kD = d.get();
    }

    public boolean hasChanged() {
        return p.hasChanged() || i.hasChanged() || d.hasChanged();
    }

    public TunableTelemetryPIDController createLoggablePIDController(String logTable) {
        return new TunableTelemetryPIDController(logTable, this);
    }
}
