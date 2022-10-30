package frc.robot.commands.drive.teleop;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.SwerveUtils;

import java.util.function.DoubleSupplier;

public class RobotOrientatedDriveCommand extends SwerveDriveCommand {

    public RobotOrientatedDriveCommand(
            DoubleSupplier xAxisSupplier, DoubleSupplier yAxisSupplier, DoubleSupplier rotationSupplier,
            SwerveDriveSubsystem driveSubsystem
    ) {
        super(xAxisSupplier, yAxisSupplier, rotationSupplier, driveSubsystem);
    }

    @Override
    public void execute() {
        double[] normalized = SwerveUtils
                .applyCircleDeadZone(xAxisSupplier.getAsDouble(), yAxisSupplier.getAsDouble(), 1.0);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                xRateLimiter.calculate(scaleXY(normalized[0])), yRateLimiter.calculate(scaleXY(normalized[1])),
                rotationLimiter.calculate(scaleRotation(rotationSupplier.getAsDouble()))
        );

        setDriveChassisSpeedsWithDeadZone(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
