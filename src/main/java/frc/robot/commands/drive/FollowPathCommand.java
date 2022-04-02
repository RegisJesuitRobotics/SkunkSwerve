package frc.robot.commands.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class FollowPathCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;
    private final PathPlannerTrajectory path;

    private final HolonomicDriveController driveController = new HolonomicDriveController(
            new PIDController(DriveTrainConstants.PATH_POSITIONAL_VELOCITY_P, 0.0, 0.0),
            new PIDController(DriveTrainConstants.PATH_POSITIONAL_VELOCITY_P, 0.0, 0.0), new ProfiledPIDController(
                    DriveTrainConstants.PATH_ANGULAR_VELOCITY_P, 0.0, 0.0, DriveTrainConstants.ANGULAR_CONSTRAINTS));

    private final Timer timer = new Timer();

    public FollowPathCommand(PathPlannerTrajectory path, SwerveDriveSubsystem driveSubsystem) {
        this.path = path;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.setOptimizeStates(true);

        driveSubsystem.resetOdometry(
                new Pose2d(path.getInitialPose().getTranslation(), path.getInitialState().holonomicRotation));

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        PathPlannerState desiredState = (PathPlannerState) path.sample(currentTime);
        ChassisSpeeds chassisSpeeds = driveController.calculate(driveSubsystem.getPose(), desiredState,
                desiredState.holonomicRotation);

        driveSubsystem.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();

        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(path.getTotalTimeSeconds());
    }
}
