package frc.robot.commands.drive.teleop;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.SwerveUtils;

import java.util.function.DoubleSupplier;

public class AlwaysFacingDriveCommand extends SwerveDriveCommand {
    private final Translation2d point;

    private final ProfiledPIDController rotationController = new ProfiledPIDController(
            DriveTrainConstants.PATH_ANGULAR_VELOCITY_P, 0.0, 0.0, DriveTrainConstants.ANGULAR_CONSTRAINTS
    );

    /**
     * @param point          the point to always be facing
     * @param xAxisSupplier  the x-axis supplier for driving
     * @param yAxisSupplier  the y-axis supplier for driving
     * @param driveSubsystem the swerve drive subsystem
     */
    public AlwaysFacingDriveCommand(
            Translation2d point, DoubleSupplier xAxisSupplier, DoubleSupplier yAxisSupplier,
            SwerveDriveSubsystem driveSubsystem
    ) {
        super(xAxisSupplier, yAxisSupplier, () -> 0, driveSubsystem);
        this.point = point;
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveSubsystem.getPose();
        Translation2d subtracted = robotPose.getTranslation().minus(point);
        Rotation2d desiredHeading = new Rotation2d(subtracted.getX(), subtracted.getY());

        double thetaVelocity = rotationController
                .calculate(robotPose.getRotation().getRadians(), desiredHeading.getRadians());

        double[] normalized = SwerveUtils.applyCircleDeadZone(
                xAxisSupplier.getAsDouble(), yAxisSupplier.getAsDouble(),
                DriveTrainConstants.MAX_TELEOP_VELOCITY_METERS_PER_SECOND
        );
        driveSubsystem.setChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        scaleXY(normalized[0]), scaleXY(normalized[1]), thetaVelocity, robotPose.getRotation()
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
