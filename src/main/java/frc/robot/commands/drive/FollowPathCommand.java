package frc.robot.commands.drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.logging.LoggablePIDController;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.function.Supplier;

public class FollowPathCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;
    private final Supplier<PathPlannerTrajectory> pathSupplier;
    private PathPlannerTrajectory currentPath;
    private final boolean shouldResetOdometry;

    private final NetworkTable baseTable = NetworkTableInstance.getDefault().getTable("followPath");
    private final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
            new LoggablePIDController(
                    "followPath/xController", baseTable.getSubTable("xController"),
                    DriveTrainConstants.PATH_TRANSLATION_POSITION_P, 0.0, 0.0
            ),
            new LoggablePIDController(
                    "followPath/yController", baseTable.getSubTable("yController"),
                    DriveTrainConstants.PATH_TRANSLATION_POSITION_P, 0.0, 0.0
            ),
            new LoggablePIDController(
                    "followPath/thetaController", baseTable.getSubTable("thetaController"),
                    DriveTrainConstants.PATH_ANGULAR_POSITION_P, 0.0, 0.0
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
        this(PathPlanner.loadPath(pathName, DriveTrainConstants.PATH_CONSTRAINTS), shouldResetOdometry, driveSubsystem);
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
        PathPlannerState desiredState = (PathPlannerState) currentPath.sample(currentTime);
        Pose2d currentPose = driveSubsystem.getPose();
        ChassisSpeeds chassisSpeeds = driveController.calculate(currentPose, desiredState);

        driveSubsystem.setChassisSpeeds(chassisSpeeds, false);

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
        return timer.hasElapsed(currentPath.getTotalTimeSeconds());
    }
}
