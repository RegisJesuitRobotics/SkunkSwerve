package frc.robot.commands.drive.teleop;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveTrainConstants;
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
        driveSubsystem.setChassisSpeeds(
                new ChassisSpeeds(
                        scaleXY(normalized[0]), scaleXY(normalized[1]),
                        rotationSupplier.getAsDouble()
                                * DriveTrainConstants.MAX_TELEOP_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                ), true
        );
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
