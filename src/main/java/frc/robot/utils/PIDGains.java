package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;

public class PIDGains {
    public final double p;
    public final double i;
    public final double d;

    public PIDGains(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public void setSlot(SlotConfiguration slot) {
        slot.kP = p;
        slot.kI = i;
        slot.kD = d;
    }
}
