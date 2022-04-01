package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

public class PIDFGains {
    public final double p;
    public final double i;
    public final double d;
    public final double f;
    public final double arbFF;

    public PIDFGains(double p, double i, double d, double f, double arbFF) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.arbFF = arbFF;
    }

    /**
     * This does NOT set the arbFF
     */
    public void setSlot(SlotConfiguration slot) {
        slot.kP = p;
        slot.kI = i;
        slot.kD = d;
        slot.kF = f;
    }
}
