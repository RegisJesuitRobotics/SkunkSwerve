package frc.robot.commands.drive.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
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
    protected final DoubleSupplier translationalMaxSpeedSupplier;
    protected final DoubleSupplier angularMaxSpeedSupplier;
    protected final SwerveDriveSubsystem driveSubsystem;

    protected final SlewRateLimiter xRateLimiter;
    protected final SlewRateLimiter yRateLimiter;
    protected final SlewRateLimiter rotationLimiter;

    protected SwerveDriveCommand(
            DoubleSupplier xAxisSupplier, DoubleSupplier yAxisSupplier, DoubleSupplier rotationSupplier,
            DoubleSupplier translationalMaxSpeedSupplier, DoubleSupplier angularMaxSpeedSupplier,
            SwerveDriveSubsystem driveSubsystem
    ) {
        this.xAxisSupplier = xAxisSupplier;
        this.yAxisSupplier = yAxisSupplier;
        this.rotationSupplier = rotationSupplier;
        this.translationalMaxSpeedSupplier = translationalMaxSpeedSupplier;
        this.angularMaxSpeedSupplier = angularMaxSpeedSupplier;
        this.driveSubsystem = driveSubsystem;

        this.xRateLimiter = new SlewRateLimiter(DriveTrainConstants.TRANSLATION_RATE_LIMIT_METERS_SECOND);
        this.yRateLimiter = new SlewRateLimiter(DriveTrainConstants.TRANSLATION_RATE_LIMIT_METERS_SECOND);
        this.rotationLimiter = new SlewRateLimiter(DriveTrainConstants.ANGULAR_RATE_LIMIT_RADIANS_SECOND);

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
        Translation2d normalized = SwerveUtils
                .applyCircleDeadZone(new Translation2d(xAxisSupplier.getAsDouble(), yAxisSupplier.getAsDouble()), 1.0);
        double[] scaled = new double[3];
        scaled[0] = xRateLimiter.calculate(scaleXY(normalized.getX(), translationalMaxSpeedSupplier.getAsDouble()));
        scaled[1] = yRateLimiter.calculate(scaleXY(normalized.getY(), translationalMaxSpeedSupplier.getAsDouble()));
        scaled[2] = rotationLimiter
                .calculate(scaleRotation(rotationSupplier.getAsDouble(), angularMaxSpeedSupplier.getAsDouble()));
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

    protected static double scaleXY(double value, double maxSpeed) {
        return Math.pow(MathUtil.applyDeadband(Math.abs(value), 0.05), 2.0) * Math.signum(value) * maxSpeed;
    }

    protected static double scaleRotation(double value, double maxSpeed) {
        return Math.pow(MathUtil.applyDeadband(Math.abs(value), 0.05), 2) * Math.signum(value) * maxSpeed;
    }
}
