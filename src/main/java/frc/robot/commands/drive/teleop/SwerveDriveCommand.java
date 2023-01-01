package frc.robot.commands.drive.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.SwerveUtils;
import frc.robot.utils.VectorRateLimiter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDriveCommand extends CommandBase {
    protected static final ChassisSpeeds zeroMovement = new ChassisSpeeds(0.0, 0.0, 0.0);
    private final DoubleSupplier xAxisSupplier;
    private final DoubleSupplier yAxisSupplier;
    private final DoubleSupplier rotationSupplier;
    private final DoubleSupplier translationalMaxSpeedSupplier;
    private final DoubleSupplier angularMaxSpeedSupplier;
    private final BooleanSupplier shouldFieldRelative;
    private final SwerveDriveSubsystem driveSubsystem;

    private final VectorRateLimiter translationLimiter;
    private final SlewRateLimiter rotationLimiter;

    public SwerveDriveCommand(
            DoubleSupplier xAxisSupplier,
            DoubleSupplier yAxisSupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier shouldFieldRelative,
            DoubleSupplier translationalMaxSpeedSupplier,
            DoubleSupplier angularMaxSpeedSupplier,
            SwerveDriveSubsystem driveSubsystem) {
        this.xAxisSupplier = xAxisSupplier;
        this.yAxisSupplier = yAxisSupplier;
        this.rotationSupplier = rotationSupplier;
        this.shouldFieldRelative = shouldFieldRelative;
        this.translationalMaxSpeedSupplier = translationalMaxSpeedSupplier;
        this.angularMaxSpeedSupplier = angularMaxSpeedSupplier;

        this.translationLimiter =
                new VectorRateLimiter(DriveTrainConstants.TRANSLATION_RATE_LIMIT_METERS_SECOND_SQUARED);
        this.rotationLimiter = new SlewRateLimiter(DriveTrainConstants.ANGULAR_RATE_LIMIT_RADIANS_SECOND_SQUARED);

        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        translationLimiter.reset();
        rotationLimiter.reset(0);
    }

    @Override
    public void execute() {
        double[] inputs = getNormalizedScaledRateLimitedXYTheta();
        ChassisSpeeds chassisSpeeds;
        if (shouldFieldRelative.getAsBoolean()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    inputs[0], inputs[1], inputs[2], driveSubsystem.getPose().getRotation());
        } else {
            chassisSpeeds = new ChassisSpeeds(inputs[0], inputs[1], inputs[2]);
        }

        if (SwerveUtils.inEpoch(
                chassisSpeeds, zeroMovement, DriveTrainConstants.TELEOP_MINIMUM_VELOCITY_METERS_PER_SECOND)) {
            driveSubsystem.stopMovement();
        } else {
            driveSubsystem.setChassisSpeeds(chassisSpeeds, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double[] getNormalizedScaledRateLimitedXYTheta() {
        Translation2d normalized = SwerveUtils.applyCircleDeadZone(
                new Translation2d(xAxisSupplier.getAsDouble(), yAxisSupplier.getAsDouble()), 1.0);
        double[] scaled = new double[3];
        Translation2d translation = translationLimiter.calculate(new Translation2d(
                scaleValue(normalized.getX(), translationalMaxSpeedSupplier.getAsDouble()),
                scaleValue(normalized.getY(), translationalMaxSpeedSupplier.getAsDouble())));
        scaled[0] = translation.getX();
        scaled[1] = translation.getY();
        scaled[2] = rotationLimiter.calculate(
                scaleValue(rotationSupplier.getAsDouble(), angularMaxSpeedSupplier.getAsDouble()));

        return scaled;
    }

    private static double scaleValue(double value, double maxSpeed) {
        return Math.pow(MathUtil.applyDeadband(Math.abs(value), 0.05), 2.0) * Math.signum(value) * maxSpeed;
    }
}
