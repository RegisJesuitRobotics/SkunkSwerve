package frc.robot.telemetry.tunable;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.telemetry.types.DoubleTelemetryEntry;

public class TunableTelemetryProfiledPIDController extends ProfiledPIDController {

    private final DoubleTelemetryEntry currentMeasurementEntry;
    private final DoubleTelemetryEntry goalPositionEntry;
    private final DoubleTelemetryEntry goalVelocityEntry;
    private final DoubleTelemetryEntry outputEntry;

    private final TunablePIDGains pidGains;
    private final TunableTrapezoidalProfileGains profileGains;

    public TunableTelemetryProfiledPIDController(
            String logTable, TunablePIDGains pidGains, TunableTrapezoidalProfileGains profileGains
    ) {
        this(logTable, pidGains, profileGains, 0.02);
    }

    public TunableTelemetryProfiledPIDController(
            String logTable, TunablePIDGains pidGains, TunableTrapezoidalProfileGains profileGains, double period
    ) {
        super(
                pidGains.p.get(), pidGains.i.get(), pidGains.d.get(),
                new Constraints(profileGains.maxVelocity.get(), profileGains.maxAcceleration.get()), period
        );
        this.pidGains = pidGains;
        this.profileGains = profileGains;

        logTable += "/";
        currentMeasurementEntry = new DoubleTelemetryEntry(logTable + "currentMeasurement", true);
        goalPositionEntry = new DoubleTelemetryEntry(logTable + "goalPosition", true);
        goalVelocityEntry = new DoubleTelemetryEntry(logTable + "goalVelocity", true);
        outputEntry = new DoubleTelemetryEntry(logTable + "output", true);
    }

    @Override
    public double calculate(double measurement) {
        if (pidGains.hasChanged() || profileGains.hasChanged()) {
            setPID(pidGains.p.get(), pidGains.i.get(), pidGains.d.get());
            setConstraints(new Constraints(profileGains.maxVelocity.get(), profileGains.maxAcceleration.get()));
        }

        currentMeasurementEntry.append(measurement);
        goalPositionEntry.append(getGoal().position);
        goalVelocityEntry.append(getGoal().velocity);

        double output = super.calculate(measurement);
        outputEntry.append(output);

        return output;
    }
}
