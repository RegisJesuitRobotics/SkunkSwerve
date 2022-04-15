package frc.robot.utils;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class PIDFFFGains extends PIDGains {
    public final SimpleMotorFeedforward feedforward;

    public PIDFFFGains(double p, double i, double d, double arbFF, double vFF, double aFF) {
        super(p, i, d);

        feedforward = new SimpleMotorFeedforward(arbFF, vFF, aFF);
    }
}
