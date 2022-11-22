package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.telemetry.LoggablePIDController;

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

    public PIDController createPIDController() {
        return new PIDController(p, i, d);
    }

    public LoggablePIDController createLoggablePIDController(String logTable) {
        return new LoggablePIDController(logTable, p, i, d);
    }
}
