package frc.robot.commands.drive.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.SwerveUtils;

import java.util.function.DoubleSupplier;

public abstract class SwerveDriveCommand extends CommandBase {
    protected static final ChassisSpeeds zeroMovement = new ChassisSpeeds(0.0, 0.0, 0.0);
    protected final DoubleSupplier xAxisSupplier;
    protected final DoubleSupplier yAxisSupplier;
    protected final DoubleSupplier rotationSupplier;
    protected final SwerveDriveSubsystem driveSubsystem;

    protected final SlewRateLimiter xRateLimiter;
    protected final SlewRateLimiter yRateLimiter;
    protected final SlewRateLimiter rotationLimiter;

    protected SwerveDriveCommand(
            DoubleSupplier xAxisSupplier, DoubleSupplier yAxisSupplier, DoubleSupplier rotationSupplier,
            SwerveDriveSubsystem driveSubsystem
    ) {
        this.xAxisSupplier = xAxisSupplier;
        this.yAxisSupplier = yAxisSupplier;
        this.rotationSupplier = rotationSupplier;
        this.driveSubsystem = driveSubsystem;

        this.xRateLimiter = new SlewRateLimiter(DriveTrainConstants.TRANSLATION_RATE_LIMIT_METERS_SECOND);
        this.yRateLimiter = new SlewRateLimiter(DriveTrainConstants.TRANSLATION_RATE_LIMIT_METERS_SECOND);
        this.rotationLimiter = new SlewRateLimiter(DriveTrainConstants.ROTATION_RATE_LIMIT_RADIANS_SECOND);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        xRateLimiter.reset(0);
        yRateLimiter.reset(0);
        rotationLimiter.reset(0);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    protected double[] getNormalizedScaledRateLimitedXYTheta() {
        double[] normalized = SwerveUtils
                .applyCircleDeadZone(xAxisSupplier.getAsDouble(), yAxisSupplier.getAsDouble(), 1.0);
        double[] scaled = new double[3];
        scaled[0] = xRateLimiter.calculate(scaleXY(normalized[0]));
        scaled[1] = yRateLimiter.calculate(scaleXY(normalized[1]));
        scaled[2] = rotationLimiter.calculate(scaleRotation(rotationSupplier.getAsDouble()));
        return scaled;
    }

    protected void setDriveChassisSpeedsWithDeadZone(ChassisSpeeds chassisSpeeds) {
        if (SwerveUtils
                .inEpoch(chassisSpeeds, zeroMovement, DriveTrainConstants.TELEOP_MINIMUM_VELOCITY_METERS_PER_SECOND)) {
            driveSubsystem.stopMovement();
        } else {
            driveSubsystem.setChassisSpeeds(chassisSpeeds, true);
        }
    }

    protected static double scaleXY(double value) {
        return Math.pow(MathUtil.applyDeadband(Math.abs(value), 0.05), 7.0 / 2) * Math.signum(value)
                * DriveTrainConstants.MAX_TELEOP_VELOCITY_METERS_PER_SECOND;
    }

    protected static double scaleRotation(double value) {
        return value * DriveTrainConstants.MAX_TELEOP_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }
}
