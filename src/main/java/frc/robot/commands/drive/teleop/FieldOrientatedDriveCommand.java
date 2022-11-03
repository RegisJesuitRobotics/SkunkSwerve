package frc.robot.commands.drive.teleop;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.function.DoubleSupplier;

public class FieldOrientatedDriveCommand extends SwerveDriveCommand {

    public FieldOrientatedDriveCommand(
            DoubleSupplier xAxisSupplier, DoubleSupplier yAxisSupplier, DoubleSupplier rotationSupplier,
            SwerveDriveSubsystem driveSubsystem
    ) {
        super(xAxisSupplier, yAxisSupplier, rotationSupplier, driveSubsystem);
    }

    @Override
    public void execute() {
        double[] inputs = getNormalizedScaledRateLimitedXYTheta();
        ChassisSpeeds chassisSpeeds = ChassisSpeeds
                .fromFieldRelativeSpeeds(inputs[0], inputs[1], inputs[2], driveSubsystem.getPose().getRotation());

        setDriveChassisSpeedsWithDeadZone(chassisSpeeds);
    }
}
