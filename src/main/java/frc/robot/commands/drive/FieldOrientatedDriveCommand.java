package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.SwerveMathUtils;

import java.util.function.DoubleSupplier;

public class FieldOrientatedDriveCommand extends CommandBase {
    private final DoubleSupplier xAxisSupplier;
    private final DoubleSupplier yAxisSupplier;
    private final DoubleSupplier rotationSupplier;
    private final SwerveDriveSubsystem driveSubsystem;

    public FieldOrientatedDriveCommand(DoubleSupplier xAxisSupplier, DoubleSupplier yAxisSupplier,
            DoubleSupplier rotationSupplier, SwerveDriveSubsystem driveSubsystem) {
        this.xAxisSupplier = xAxisSupplier;
        this.yAxisSupplier = yAxisSupplier;
        this.rotationSupplier = rotationSupplier;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double[] normalized = SwerveMathUtils.applyCircleDeadZone(xAxisSupplier.getAsDouble(),
                yAxisSupplier.getAsDouble(), 1.0);
        driveSubsystem.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                normalized[0] * DriveTrainConstants.MAX_TELEOP_VELOCITY_METERS_PER_SECOND,
                normalized[1] * DriveTrainConstants.MAX_TELEOP_VELOCITY_METERS_PER_SECOND,
                rotationSupplier.getAsDouble() * DriveTrainConstants.MAX_TELEOP_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                driveSubsystem.getGyroRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
