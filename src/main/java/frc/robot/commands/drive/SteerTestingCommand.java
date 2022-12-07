package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class SteerTestingCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;

    public SteerTestingCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.getField2d().getObject("SteerTesting").setPose(new Pose2d(15.980 / 2.0, 8.210 / 2, Rotation2d.fromDegrees(0.0)));
    }

    @Override
    public void execute() {
        Rotation2d angle = driveSubsystem.getField2d().getObject("SteerTesting").getPose().getRotation();

        SwerveModuleState[] states = new SwerveModuleState[DriveTrainConstants.NUM_MODULES];

        for (int i = 0; i < DriveTrainConstants.NUM_MODULES; i++) {
            states[i] = new SwerveModuleState(0.0, angle);
        }

        driveSubsystem.setRawStates(true, true, states);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
//        driveSubsystem.getField2d().getObject("SteerTesting").close();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
