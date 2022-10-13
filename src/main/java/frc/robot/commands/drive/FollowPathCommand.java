package frc.robot.commands.drive;

import com.pathplanner.lib.PathPlanner;
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
    private final boolean shouldResetOdometry;

    private final HolonomicDriveController driveController = new HolonomicDriveController(
            new PIDController(DriveTrainConstants.PATH_POSITIONAL_VELOCITY_P, 0.0, 0.0),
            new PIDController(DriveTrainConstants.PATH_POSITIONAL_VELOCITY_P, 0.0, 0.0),
            new ProfiledPIDController(
                    DriveTrainConstants.PATH_ANGULAR_VELOCITY_P, 0.0, 0.0, DriveTrainConstants.ANGULAR_CONSTRAINTS
            )
    );

    private final Timer timer = new Timer();

    /**
     * A follow path command made with the path name
     *
     * @param pathName            the name of the path in path planner
     * @param shouldResetOdometry if odometry should be reset to the position at the
     *                            beginning of the path
     * @param driveSubsystem      the swerve drive subsystem
     */
    public FollowPathCommand(String pathName, boolean shouldResetOdometry, SwerveDriveSubsystem driveSubsystem) {
        this(
                PathPlanner.loadPath(
                        pathName, DriveTrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                        DriveTrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
                ), shouldResetOdometry, driveSubsystem
        );
    }

    /**
     * A follow path command made with the trajectory
     *
     * @param path                the trajectory
     * @param shouldResetOdometry if odometry should be reset to the position at the
     *                            beginning of the path
     * @param driveSubsystem      the swerve drive subsystem
     */
    public FollowPathCommand(
            PathPlannerTrajectory path, boolean shouldResetOdometry, SwerveDriveSubsystem driveSubsystem
    ) {
        this.path = path;
        this.shouldResetOdometry = shouldResetOdometry;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        if (shouldResetOdometry) {
            driveSubsystem.resetOdometry(
                    new Pose2d(path.getInitialPose().getTranslation(), path.getInitialState().holonomicRotation)
            );
        }

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        PathPlannerState desiredState = (PathPlannerState) path.sample(currentTime);
        ChassisSpeeds chassisSpeeds = driveController
                .calculate(driveSubsystem.getPose(), desiredState, desiredState.holonomicRotation);

        driveSubsystem.setChassisSpeeds(chassisSpeeds, false);
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
