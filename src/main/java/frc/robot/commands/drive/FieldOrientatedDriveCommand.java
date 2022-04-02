package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.joysticks.ThrustMaster;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class FieldOrientatedDriveCommand extends CommandBase {
    private final ThrustMaster thrustMaster;
    private final SwerveDriveSubsystem driveSubsystem;

    public FieldOrientatedDriveCommand(ThrustMaster thrustMaster, SwerveDriveSubsystem driveSubsystem) {
        this.thrustMaster = thrustMaster;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.setOptimizeStates(true);
    }

    @Override
    public void execute() {
        driveSubsystem.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                thrustMaster.stick.getXAxis() * DriveTrainConstants.MAX_TELEOP_VELOCITY_METERS_PER_SECOND,
                thrustMaster.stick.getYAxis() * DriveTrainConstants.MAX_TELEOP_VELOCITY_METERS_PER_SECOND,
                thrustMaster.stick.getZAxis() * DriveTrainConstants.MAX_TELEOP_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
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
