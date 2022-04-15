package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.SwerveMathUtils;

import java.util.function.DoubleSupplier;

public class AlwaysFacingDriveCommand extends CommandBase {
    private final Translation2d point;
    private final DoubleSupplier xAxisSupplier;
    private final DoubleSupplier yAxisSupplier;

    private final SwerveDriveSubsystem driveSubsystem;

    private final ProfiledPIDController rotationController = new ProfiledPIDController(
            DriveTrainConstants.PATH_ANGULAR_VELOCITY_P, 0.0, 0.0, DriveTrainConstants.ANGULAR_CONSTRAINTS
    );

    public AlwaysFacingDriveCommand(
            Translation2d point, DoubleSupplier xAxisSupplier, DoubleSupplier yAxisSupplier,
            SwerveDriveSubsystem driveSubsystem
    ) {
        this.point = point;
        this.xAxisSupplier = xAxisSupplier;
        this.yAxisSupplier = yAxisSupplier;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        Pose2d robotPose = driveSubsystem.getPose();
        Translation2d subtracted = robotPose.getTranslation().minus(point);
        Rotation2d desiredHeading = new Rotation2d(subtracted.getX(), subtracted.getY());

        double thetaVelocity = rotationController
                .calculate(robotPose.getRotation().getRadians(), desiredHeading.getRadians());

        double[] normalized = SwerveMathUtils.applyCircleDeadZone(
                xAxisSupplier.getAsDouble(), yAxisSupplier.getAsDouble(),
                DriveTrainConstants.MAX_TELEOP_VELOCITY_METERS_PER_SECOND
        );
        driveSubsystem.setChassisSpeeds(
                ChassisSpeeds
                        .fromFieldRelativeSpeeds(normalized[0], normalized[1], thetaVelocity, robotPose.getRotation()),
                true
        );
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
