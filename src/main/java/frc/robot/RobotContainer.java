package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MiscConstants;
import frc.robot.commands.drive.FollowPathCommand;
import frc.robot.commands.drive.HoldDrivePositionCommand;
import frc.robot.commands.drive.SetModuleRotationCommand;
import frc.robot.commands.drive.teleop.FieldOrientatedDriveCommand;
import frc.robot.commands.drive.teleop.RobotOrientatedDriveCommand;
import frc.robot.commands.util.InstantRunWhenDisabledCommand;
import frc.robot.joysticks.ThrustMaster;
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

    private final ThrustMaster driverController = new ThrustMaster(0);

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

        new Trigger(autoCommandChooser::hasNewValue).whenActive(
                new InstantRunWhenDisabledCommand(
                        () -> noAutoSelectedAlert.set(autoCommandChooser.getSelected() == null)
                )
        );

        Shuffleboard.getTab("DriveTrainRaw").add("Auto Chooser", autoCommandChooser);
    }

    private void configureButtonBindings() {
        driveCommandChooser.setDefaultOption(
                "Field Orientated",
                new FieldOrientatedDriveCommand(
                        () -> -driverController.stick.getYAxis(), () -> -driverController.stick.getXAxis(),
                        driverController.stick::getZAxis, driveSubsystem
                )
        );
        driveCommandChooser.addOption(
                "Robot Orientated",
                new RobotOrientatedDriveCommand(
                        () -> -driverController.stick.getYAxis(), () -> -driverController.stick.getXAxis(),
                        driverController.stick::getZAxis, driveSubsystem
                )
        );

        ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrainRaw");
        driveTab.add("Drive Style", driveCommandChooser);

        new Trigger(driveCommandChooser::hasNewValue).whenActive(
                new InstantRunWhenDisabledCommand(() -> evaluateDriveStyle(driveCommandChooser.getSelected()))
        );

        driverController.buttonOne.whenPressed(new InstantRunWhenDisabledCommand(driveSubsystem::zeroHeading));
        driverController.buttonTwo.whenHeld(new SetModuleRotationCommand(0.0, driveSubsystem));
        driverController.buttonThree.whileHeld(new HoldDrivePositionCommand(driveSubsystem));
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
