package frc.robot.commands.drive.teleop;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class HybridOrientatedDriveCommand extends SwerveDriveCommand {
    private final BooleanSupplier shouldFieldRelative;

    public HybridOrientatedDriveCommand(
            DoubleSupplier xAxisSupplier, DoubleSupplier yAxisSupplier, DoubleSupplier rotationSupplier,
            BooleanSupplier shouldFieldRelative, DoubleSupplier translationalMaxSpeedSupplier,
            DoubleSupplier angularMaxSpeedSupplier, SwerveDriveSubsystem driveSubsystem
    ) {
        super(
                xAxisSupplier, yAxisSupplier, rotationSupplier, translationalMaxSpeedSupplier, angularMaxSpeedSupplier,
                driveSubsystem
        );
        this.shouldFieldRelative = shouldFieldRelative;
    }

    @Override
    public void execute() {
        double[] inputs = getNormalizedScaledRateLimitedXYTheta();
        ChassisSpeeds chassisSpeeds;
        if (shouldFieldRelative.getAsBoolean()) {
            chassisSpeeds = ChassisSpeeds
                    .fromFieldRelativeSpeeds(inputs[0], inputs[1], inputs[2], driveSubsystem.getPose().getRotation());
        } else {
            chassisSpeeds = new ChassisSpeeds(inputs[0], inputs[1], inputs[2]);
        }

        setDriveChassisSpeedsWithDeadZone(chassisSpeeds);
    }
}
