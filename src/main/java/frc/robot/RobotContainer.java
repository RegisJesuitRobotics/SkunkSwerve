package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.commands.drive.FollowPathCommand;
import frc.robot.commands.drive.HoldDrivePositionCommand;
import frc.robot.commands.drive.characterize.DynamicCharacterizeDriveCommand;
import frc.robot.commands.drive.characterize.QuasistaticCharacterizeDriveCommand;
import frc.robot.commands.drive.teleop.HybridOrientatedDriveCommand;
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

    private final CommandXboxController driverController = new CommandXboxController(0);

    private final ListenableSendableChooser<Command> driveCommandChooser = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<Command> autoCommandChooser = new ListenableSendableChooser<>();
    private final Alert noAutoSelectedAlert = new Alert("No Auto Routine Selected", AlertType.WARNING);

    public RobotContainer() {
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
        autoCommandChooser.addOption(
                "QuasistaticCharacterizeDriveCommand", new QuasistaticCharacterizeDriveCommand(0.2, driveSubsystem)
        );
        autoCommandChooser
                .addOption("DynamicCharacterizeDriveCommand", new DynamicCharacterizeDriveCommand(8.0, driveSubsystem));

        new Trigger(autoCommandChooser::hasNewValue).onTrue(
                new InstantCommand(() -> noAutoSelectedAlert.set(autoCommandChooser.getSelected() == null))
                        .ignoringDisable(true)
        );

        // Shuffleboard.getTab("DriveTrainRaw").add("Auto Chooser", autoCommandChooser);
    }

    private void configureButtonBindings() {
        driveCommandChooser.setDefaultOption(
                "Hybrid (Default to Field Relative but use robot centric when holding button)",
                new HybridOrientatedDriveCommand(
                        () -> -driverController.getLeftX(), () -> -driverController.getLeftY(),
                        () -> -driverController.getRightX(), driverController.rightBumper().negate(), driveSubsystem
                )
        );
        driveCommandChooser.addOption(
                "Field Orientated",
                new HybridOrientatedDriveCommand(
                        () -> -driverController.getLeftX(), () -> -driverController.getLeftY(),
                        () -> -driverController.getRightX(), () -> true, driveSubsystem
                )
        );
        driveCommandChooser.addOption(
                "Robot Orientated",
                new HybridOrientatedDriveCommand(
                        () -> -driverController.getLeftX(), () -> -driverController.getLeftY(),
                        () -> -driverController.getRightX(), () -> false, driveSubsystem
                )
        );

        ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrainRaw");
        driveTab.add("Drive Style", driveCommandChooser);

        new Trigger(driveCommandChooser::hasNewValue).onTrue(
                new InstantCommand(() -> evaluateDriveStyle(driveCommandChooser.getSelected())).ignoringDisable(true)
        );

        driverController.b().onTrue(new InstantCommand(driveSubsystem::resetOdometry).ignoringDisable(true));
        driverController.leftBumper().whileTrue(new HoldDrivePositionCommand(driveSubsystem).repeatedly());
        driverController.x().debounce(0.5).onTrue(new FollowPathCommand(() -> {
            Pose2d currentPose = driveSubsystem.getPose();
            Pose2d targetPose = new Pose2d();
            Translation2d translation = currentPose.minus(targetPose).getTranslation();
            return PathPlanner.generatePath(
                    DriveTrainConstants.PATH_CONSTRAINTS,
                    new PathPoint(
                            currentPose.getTranslation(),
                            new Rotation2d(translation.getX(), translation.getY()).unaryMinus(),
                            currentPose.getRotation()
                    ),
                    new PathPoint(
                            new Translation2d(0, 0), new Rotation2d(translation.getX(), translation.getY()),
                            new Rotation2d(0)
                    )
            );
        }, false, driveSubsystem).until(driverController.rightBumper()));
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
