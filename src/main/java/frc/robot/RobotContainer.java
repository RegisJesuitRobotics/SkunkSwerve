package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.commands.drive.CharacterizeDriveCommand;
import frc.robot.commands.drive.FollowPathCommand;
import frc.robot.commands.drive.HoldDrivePositionCommand;
import frc.robot.commands.drive.teleop.HybridOrientatedDriveCommand;
import frc.robot.commands.util.InstantRunWhenDisabledCommand;
import frc.robot.joysticks.PseudoXboxController;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ListenableSendableChooser;



/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SwerveDriveSubsystem driveSubsystem = new SwerveDriveSubsystem();

    private final PseudoXboxController driverController = new PseudoXboxController(0);

    private final ListenableSendableChooser<Command> driveCommandChooser = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<Command> autoCommandChooser = new ListenableSendableChooser<>();
    private final Alert noAutoSelectedAlert = new Alert("No Auto Routine Selected", AlertType.WARNING);

    public RobotContainer() {
        NetworkTableInstance.getDefault().setUpdateRate(0.01);
        configureButtonBindings();
        configureAutos();
    }

    private void configureAutos() {
        if (MiscConstants.enablePathPlannerServer) {
            PathPlannerServer.startServer(5810);
        }

        autoCommandChooser.setDefaultOption("Nothing", null);
        autoCommandChooser.addOption("UpDownWithRotation", new FollowPathCommand("WithRotation", true, driveSubsystem));
        autoCommandChooser.addOption("UpDownNoRotation", new FollowPathCommand("NoRotation", true, driveSubsystem));
        autoCommandChooser
                .addOption("StraightWithRotation", new FollowPathCommand("StraightWithRotation", true, driveSubsystem));
        autoCommandChooser
                .addOption("StraightNoRotation", new FollowPathCommand("StraightNoRotation", true, driveSubsystem));
        autoCommandChooser.addOption("FigureEights", new FollowPathCommand("FigureEights", true, driveSubsystem));
        autoCommandChooser.addOption(
                "FigureEightsWithRotation", new FollowPathCommand("FigureEightsWithRotation", true, driveSubsystem)
        );
        autoCommandChooser.addOption("FUN", new FollowPathCommand("FUN", true, driveSubsystem));
        autoCommandChooser.addOption("CharacterizeDriveTrain", new CharacterizeDriveCommand(driveSubsystem));

        new Trigger(autoCommandChooser::hasNewValue).whenActive(
                new InstantRunWhenDisabledCommand(
                        () -> noAutoSelectedAlert.set(autoCommandChooser.getSelected() == null)
                )
        );

        Shuffleboard.getTab("DriveTrainRaw").add("Auto Chooser", autoCommandChooser);
    }

    private void configureButtonBindings() {
        driveCommandChooser.setDefaultOption(
                "Hybrid (Default to Field Relative but use robot centric when holding button)",
                new HybridOrientatedDriveCommand(
                        () -> -driverController.leftThumb.getYAxis(), () -> -driverController.leftThumb.getXAxis(),
                        () -> -driverController.rightThumb.getXAxis(), () -> !driverController.rightButton.get(),
                        driveSubsystem
                )
        );
        driveCommandChooser.addOption(
                "Field Orientated",
                new HybridOrientatedDriveCommand(
                        () -> -driverController.leftThumb.getYAxis(), () -> -driverController.leftThumb.getXAxis(),
                        () -> -driverController.rightThumb.getXAxis(), () -> true, driveSubsystem
                )
        );
        driveCommandChooser.addOption(
                "Robot Orientated",
                new HybridOrientatedDriveCommand(
                        () -> -driverController.leftThumb.getYAxis(), () -> -driverController.leftThumb.getXAxis(),
                        () -> -driverController.rightThumb.getXAxis(), () -> false, driveSubsystem
                )
        );

        ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrainRaw");
        driveTab.add("Drive Style", driveCommandChooser);

        new Trigger(driveCommandChooser::hasNewValue).whenActive(
                new InstantRunWhenDisabledCommand(() -> evaluateDriveStyle(driveCommandChooser.getSelected()))
        );

        driverController.circle.whenPressed(new InstantRunWhenDisabledCommand(driveSubsystem::resetOdometry));
        driverController.leftButton.whileHeld(new HoldDrivePositionCommand(driveSubsystem));
        driverController.square.debounce(0.5).whenActive(new FollowPathCommand(() -> {
            Pose2d currentPose = driveSubsystem.getPose();
            return PathPlanner.generatePath(
                    DriveTrainConstants.PATH_CONSTRAINTS,
                    new PathPoint(
                            currentPose.getTranslation(),
                            new Rotation2d(currentPose.getX(), currentPose.getY()).unaryMinus(),
                            currentPose.getRotation()
                    ),
                    new PathPoint(
                            new Translation2d(0, 0), new Rotation2d(currentPose.getX(), currentPose.getY()),
                            new Rotation2d(0)
                    )
            );
        }, false, driveSubsystem));
    }

    private void evaluateDriveStyle(Command newCommand) {
        Command oldCommand = driveSubsystem.getDefaultCommand();

        // Check if they are the same
        // we use the == operator instead of Command#equals() because we want to know if
        // it is the exact same object in memory
        if (newCommand == oldCommand) {
            return;
        }
        driveSubsystem.setDefaultCommand(newCommand);
        if (oldCommand != null) {
            // We have to cancel the command so the new default one will run
            oldCommand.cancel();
        }
    }

    public Command getAutonomousCommand() {
        return autoCommandChooser.getSelected();
    }
}
