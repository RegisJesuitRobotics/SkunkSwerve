package frc.robot.utils;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class TunablePIDFFFGains extends TunablePIDGains {
    public final TunableDouble arbFF;
    public final TunableDouble vFF;
    public final TunableDouble aFF;

    /**
     * @param networkName the name to use for network tables
     * @param p           the p gain
     * @param i           the i gain
     * @param d           the d gain
     * @param arbFF       the arbitrary/static feedforward (also known as kS) (in
     *                    volts per meter/second)
     * @param vFF         the velocity feedforward (also known as kV) (in volts per
     *                    meter/second)
     * @param aFF         the acceleration feedforward (also known as kA) (in volts
     *                    meter/second^2)
     */
    public TunablePIDFFFGains(String networkName, double p, double i, double d, double arbFF, double vFF, double aFF) {
        super(networkName, p, i, d);
        networkName += "/";
        this.arbFF = new TunableDouble(networkName + "arbFF", arbFF);
        this.vFF = new TunableDouble(networkName + "vFF", vFF);
        this.aFF = new TunableDouble(networkName + "aFF", aFF);
    }

    @Override
    public boolean hasChanged() {
        return super.hasChanged() || arbFF.hasChanged() || vFF.hasChanged() || aFF.hasChanged();
    }

    public SimpleMotorFeedforward getFeedforward() {
        return new SimpleMotorFeedforward(arbFF.get(), vFF.get(), aFF.get());
    }
}
