package frc.robot.commands.drive.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.function.DoubleSupplier;

public abstract class SwerveDriveCommand extends CommandBase {
    protected final DoubleSupplier xAxisSupplier;
    protected final DoubleSupplier yAxisSupplier;
    protected final DoubleSupplier rotationSupplier;
    protected final SwerveDriveSubsystem driveSubsystem;

    protected SwerveDriveCommand(
            DoubleSupplier xAxisSupplier, DoubleSupplier yAxisSupplier, DoubleSupplier rotationSupplier,
            SwerveDriveSubsystem driveSubsystem
    ) {
        this.xAxisSupplier = xAxisSupplier;
        this.yAxisSupplier = yAxisSupplier;
        this.rotationSupplier = rotationSupplier;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    protected static double scaleXY(double value) {
        return Math.pow(MathUtil.applyDeadband(Math.abs(value), 0.05), 7.0 / 2) * Math.signum(value)
                * DriveTrainConstants.MAX_TELEOP_VELOCITY_METERS_PER_SECOND;
    }

    protected static double scaleRotation(double value) {
        return value * DriveTrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }
}
