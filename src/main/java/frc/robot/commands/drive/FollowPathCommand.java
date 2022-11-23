package frc.robot.commands.drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.function.Supplier;

public class FollowPathCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;
    private final Supplier<PathPlannerTrajectory> pathSupplier;
    private PathPlannerTrajectory currentPath;
    private final boolean shouldResetOdometry;

    private final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
            AutoConstants.PATH_TRANSLATION_POSITION_GAINS.createLoggablePIDController("followPath/xController"),
            AutoConstants.PATH_TRANSLATION_POSITION_GAINS.createLoggablePIDController("followPath/yController"),
            AutoConstants.PATH_ANGULAR_POSITION_GAINS.createLoggablePIDController("followPath/thetaController")
    );

    private final PPHolonomicDriveController nextDriveController = new PPHolonomicDriveController(
            AutoConstants.PATH_TRANSLATION_POSITION_GAINS.createPIDController(),
            AutoConstants.PATH_TRANSLATION_POSITION_GAINS.createPIDController(),
            AutoConstants.PATH_ANGULAR_POSITION_GAINS.createPIDController()
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
        this(PathPlanner.loadPath(pathName, AutoConstants.PATH_CONSTRAINTS), shouldResetOdometry, driveSubsystem);
    }

    public FollowPathCommand(
            PathPlannerTrajectory path, boolean shouldResetOdometry, SwerveDriveSubsystem driveSubsystem
    ) {
        this(() -> path, shouldResetOdometry, driveSubsystem);
    }

    /**
     * A follow path command made with the trajectory
     *
     * @param pathSupplier        the trajectory
     * @param shouldResetOdometry if odometry should be reset to the position at the
     *                            beginning of the path
     * @param driveSubsystem      the swerve drive subsystem
     */
    public FollowPathCommand(
            Supplier<PathPlannerTrajectory> pathSupplier, boolean shouldResetOdometry,
            SwerveDriveSubsystem driveSubsystem
    ) {
        this.pathSupplier = pathSupplier;
        this.shouldResetOdometry = shouldResetOdometry;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        currentPath = pathSupplier.get();
        if (shouldResetOdometry) {
            driveSubsystem.resetOdometry(currentPath.getInitialHolonomicPose());
        }
        if (MiscConstants.enablePathPlannerServer) {
            PathPlannerServer.sendActivePath(currentPath.getStates());
        }
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        Pose2d currentPose = driveSubsystem.getPose();

        PathPlannerState desiredState = (PathPlannerState) currentPath.sample(currentTime);
        ChassisSpeeds chassisSpeeds = driveController.calculate(currentPose, desiredState);

        PathPlannerState nextDesiredState = (PathPlannerState) currentPath.sample(currentTime + 0.02);
        Pose2d assumedNextPose = currentPose
                .plus(
                        new Transform2d(
                                new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                                        .times(0.02),
                                new Rotation2d(chassisSpeeds.omegaRadiansPerSecond).times(0.02)
                        )
                );
        ChassisSpeeds nextChassisSpeeds = nextDriveController.calculate(assumedNextPose, nextDesiredState);

        driveSubsystem.setChassisSpeeds(chassisSpeeds, nextChassisSpeeds, false);

        if (MiscConstants.enablePathPlannerServer) {
            PathPlannerServer.sendPathFollowingData(
                    new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation), currentPose
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            driveSubsystem.stopMovement();
        }

        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(currentPath.getTotalTimeSeconds()) && driveController.atReference();
    }
}
