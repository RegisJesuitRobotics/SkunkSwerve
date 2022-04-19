package frc.robot.utils;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class PIDFFFGains extends PIDGains {
    public final SimpleMotorFeedforward feedforward;

    /**
     * @param p     the p gain
     * @param i     the i gain
     * @param d     the d gain
     * @param arbFF the arbitrary feedforward (also known as kS) (in volts)
     * @param vFF   the velocity feedforward (also known as kV) (in volts)
     * @param aFF   the acceleration feedforward (also known as kA) (in volts)
     */
    public PIDFFFGains(double p, double i, double d, double arbFF, double vFF, double aFF) {
        super(p, i, d);

        feedforward = new SimpleMotorFeedforward(arbFF, vFF, aFF);
    }
}
