package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.commands.drive.auto.Autos;
import frc.robot.commands.drive.auto.FollowPathCommand;
import frc.robot.commands.drive.teleop.SwerveDriveCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ListenableSendableChooser;

import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.atomic.AtomicBoolean;
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
    private final TeleopControlsStateManager teleopControlsStateManager = new TeleopControlsStateManager();

    private final ListenableSendableChooser<Command> driveCommandChooser = new ListenableSendableChooser<>();
    private final ListenableSendableChooser<Command> autoCommandChooser = new ListenableSendableChooser<>();
    private final Alert noAutoSelectedAlert = new Alert("No Auto Routine Selected", AlertType.WARNING);

    public RobotContainer() {
        configureButtonBindings();
        configureAutos();
    }

    private void configureAutos() {
        if (MiscConstants.TUNING_MODE) {
            PathPlannerServer.startServer(5810);
        }

        autoCommandChooser.setDefaultOption("Nothing", null);
        Autos autos = new Autos(driveSubsystem);
        for (Entry<String, Command> auto : autos.getAutos().entrySet()) {
            autoCommandChooser.addOption(auto.getKey(), auto.getValue());
        }

        new Trigger(autoCommandChooser::hasNewValue).onTrue(
                Commands.runOnce(() -> noAutoSelectedAlert.set(autoCommandChooser.getSelected() == null))
                        .ignoringDisable(true).withName("Auto Alert Checker")
        );

        Shuffleboard.getTab("DriveTrainRaw").add("Auto Chooser", autoCommandChooser);
    }

    private void configureButtonBindings() {
        GenericEntry maxTranslationSpeedEntry = Shuffleboard.getTab("DriveTrainRaw")
                .add("Max Translational Speed (Percent)", 0.9).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1.0)).getEntry();
        GenericEntry maxAngularSpeedEntry = Shuffleboard.getTab("DriveTrainRaw").add("Max Angular Speed (Percent)", 1.0)
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1.0)).getEntry();

        DoubleSupplier translationalMaxSpeedSuppler = () -> maxTranslationSpeedEntry.getDouble(0.9)
                * DriveTrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
        DoubleSupplier angularMaxSpeedSupplier = () -> maxAngularSpeedEntry.getDouble(1.0)
                * DriveTrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

        ProfiledPIDController snapController = new TunableTelemetryProfiledPIDController(
                "drive/snapController", AutoConstants.PATH_ANGULAR_POSITION_PID_GAINS,
                AutoConstants.PATH_ANGULAR_POSITION_TRAPEZOIDAL_GAINS
        );
        AtomicBoolean isSnapControllerEnabled = new AtomicBoolean(false);

        driveCommandChooser.setDefaultOption(
                "Hybrid (Default to Field Relative but use robot centric when holding button)",
                new SwerveDriveCommand(() -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> {
                    if (driverController.leftBumper().getAsBoolean()) {
                        if (isSnapControllerEnabled.getAndSet(true)) {
                            snapController.reset(driveSubsystem.getPose().getRotation().getRadians());
                        }
                        return snapController.calculate(driveSubsystem.getPose().getRotation().getRadians(), 0);
                    } else {
                        isSnapControllerEnabled.set(false);
                        return -driverController.getRightX();
                    }
                }, driverController.rightBumper().negate(), translationalMaxSpeedSuppler, angularMaxSpeedSupplier,
                        driveSubsystem
                )
        );
        driveCommandChooser.addOption(
                "Robot Orientated",
                new SwerveDriveCommand(
                        () -> -driverController.getLeftY(), () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(), () -> false, translationalMaxSpeedSuppler,
                        angularMaxSpeedSupplier, driveSubsystem
                )
        );

        SmartDashboard.putData(CommandScheduler.getInstance());
        ProfiledPIDController alwaysFacingAngularController = new TunableTelemetryProfiledPIDController(
                "drive/alwaysFacingController", AutoConstants.PATH_ANGULAR_POSITION_PID_GAINS,
                AutoConstants.PATH_ANGULAR_POSITION_TRAPEZOIDAL_GAINS
        );
        alwaysFacingAngularController.enableContinuousInput(-Math.PI, Math.PI);
        driveCommandChooser.addOption(
                "Always Facing (0, 0)",
                // This isn't optimal so if we were to actually use this in season we would have
                // some FF with where we predict we will be and use a ProfiledPIDController
                new SwerveDriveCommand(() -> -driverController.getLeftX(), () -> -driverController.getLeftY(), () -> {
                    Pose2d robotPose = driveSubsystem.getPose();
                    Translation2d targetToCurrent = new Translation2d().minus(robotPose.getTranslation());

                    return alwaysFacingAngularController
                            .calculate(robotPose.getRotation().getRadians(), targetToCurrent.getAngle().getRadians());
                }, driverController.rightBumper().negate(), translationalMaxSpeedSuppler,
                        () -> DriveTrainConstants.ANGULAR_RATE_LIMIT_RADIANS_SECOND_SQUARED, driveSubsystem
                )
        );

        ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrainRaw");
        driveTab.add("Drive Style", driveCommandChooser);

        evaluateDriveStyle(driveCommandChooser.getSelected());
        new Trigger(driveCommandChooser::hasNewValue).onTrue(
                Commands.runOnce(() -> evaluateDriveStyle(driveCommandChooser.getSelected())).ignoringDisable(true)
                        .withName("Drive Style Checker")
        );

        driverController.b().onTrue(
                Commands.runOnce(driveSubsystem::resetOdometry).ignoringDisable(true).withName("Reset Odometry")
        );
        driverController.leftBumper()
                .whileTrue(new LockModulesCommand(driveSubsystem).repeatedly().withName("Lock Modules"));

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
        }, driveSubsystem).until(driverController.rightBumper()).withName("To (0, 0) Follow Path"));
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
