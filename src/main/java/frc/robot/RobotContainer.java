package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.commands.drive.auto.Autos;
import frc.robot.commands.drive.auto.FollowPathCommand;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.commands.drive.teleop.HybridOrientatedDriveCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.Alert;
import frc.robot.utils.ListenableSendableChooser;
import frc.robot.utils.Alert.AlertType;

import java.util.Map;
import java.util.Map.Entry;
import java.util.function.DoubleSupplier;



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
        Autos autos = new Autos(driveSubsystem);
        for (Entry<String, Command> auto : autos.getAutos().entrySet()) {
            autoCommandChooser.addOption(auto.getKey(), auto.getValue());
        }

        new Trigger(autoCommandChooser::hasNewValue).onTrue(
                Commands.runOnce(() -> noAutoSelectedAlert.set(autoCommandChooser.getSelected() == null))
                        .withName("Auto Alert Checker").ignoringDisable(true)
        );

        Shuffleboard.getTab("DriveTrainRaw").add("Auto Chooser", autoCommandChooser);
    }

    private void configureButtonBindings() {
        GenericEntry maxTranslationSpeedEntry = Shuffleboard.getTab("DriveTrainRaw")
                .add("Max Translational Speed (Percent)", 1.0).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)).getEntry();
        GenericEntry maxAngularSpeedEntry = Shuffleboard.getTab("DriveTrainRaw").add("Max Angular Speed (Percent)", 1.0)
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1.0)).getEntry();
        DoubleSupplier translationalMaxSpeedSuppler = () -> maxTranslationSpeedEntry.getDouble(0.9)
                * DriveTrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
        DoubleSupplier angularMaxSpeedSupplier = () -> maxAngularSpeedEntry.getDouble(1.0)
                * DriveTrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

        driveCommandChooser.setDefaultOption(
                "Hybrid (Default to Field Relative but use robot centric when holding button)",
                new HybridOrientatedDriveCommand(
                        () -> -driverController.getLeftX(), () -> -driverController.getLeftY(),
                        () -> -driverController.getRightX(), driverController.rightBumper().negate(),
                        translationalMaxSpeedSuppler, angularMaxSpeedSupplier, driveSubsystem
                )
        );
        driveCommandChooser.addOption(
                "Field Orientated",
                new HybridOrientatedDriveCommand(
                        () -> -driverController.getLeftX(), () -> -driverController.getLeftY(),
                        () -> -driverController.getRightX(), () -> true, translationalMaxSpeedSuppler,
                        angularMaxSpeedSupplier, driveSubsystem
                )
        );
        driveCommandChooser.addOption(
                "Robot Orientated",
                new HybridOrientatedDriveCommand(
                        () -> -driverController.getLeftX(), () -> -driverController.getLeftY(),
                        () -> -driverController.getRightX(), () -> false, translationalMaxSpeedSuppler,
                        angularMaxSpeedSupplier, driveSubsystem
                )
        );

        ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrainRaw");
        driveTab.add("Drive Style", driveCommandChooser);

        new Trigger(driveCommandChooser::hasNewValue).onTrue(
                Commands.runOnce(() -> evaluateDriveStyle(driveCommandChooser.getSelected()))
                        .withName("Drive Style Checker").ignoringDisable(true)
        );

        driverController.b().onTrue(
                Commands.runOnce(driveSubsystem::resetOdometry).withName("Reset Odometry").ignoringDisable(true)
        );
        driverController.leftBumper().whileTrue(new LockModulesCommand(driveSubsystem).repeatedly());

        driverController.x().debounce(0.5).onTrue(new FollowPathCommand(() -> {
            Pose2d currentPose = driveSubsystem.getPose();
            Pose2d targetPose = new Pose2d();
            Translation2d translation = currentPose.minus(targetPose).getTranslation();
            return PathPlanner.generatePath(
                    AutoConstants.PATH_CONSTRAINTS,
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
