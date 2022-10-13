// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drive.*;
import frc.robot.commands.drive.teleop.FieldOrientatedDriveCommand;
import frc.robot.commands.drive.teleop.RobotOrientatedDriveCommand;
import frc.robot.commands.util.NameableInstantRunWhenDisabledCommand;
import frc.robot.joysticks.ThrustMaster;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;



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

    private final SendableChooser<Command> driveCommandChooser = new SendableChooser<>();
    private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();
    private final Alert noAutoSelectedAlert = new Alert("No Auto Routine Selected", AlertType.WARNING);

    public RobotContainer() {
        configureButtonBindings();
        configureAutos();
    }

    private void configureAutos() {
        autoCommandChooser.setDefaultOption("Nothing", new InstantCommand());
        autoCommandChooser.addOption("UpDownWithRotation", new FollowPathCommand("WithRotation", true, driveSubsystem));
        autoCommandChooser.addOption("UpDownNoRotation", new FollowPathCommand("NoRotation", true, driveSubsystem));
        autoCommandChooser
                .addOption("StraightWithRotation", new FollowPathCommand("StraightWithRotation", true, driveSubsystem));
        autoCommandChooser
                .addOption("StraightNoRotation", new FollowPathCommand("StraightNoRotation", true, driveSubsystem));
        autoCommandChooser.addOption("FigureEights", new FollowPathCommand("FigureEights", true, driveSubsystem));

        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("DriveTrainRaw")
                .getSubTable("Auto Chooser").getEntry("selected").addListener(
                        (EntryNotification notification) -> noAutoSelectedAlert
                                .set(notification.value.getString().equals("Nothing")),
                        EntryListenerFlags.kImmediate | EntryListenerFlags.kLocal | EntryListenerFlags.kUpdate
                                | EntryListenerFlags.kNew
                );
        Shuffleboard.getTab("DriveTrainRaw").add("Auto Chooser", autoCommandChooser);
    }

    private void configureButtonBindings() {
        driveCommandChooser.setDefaultOption(
                "Field Orientated",
                new FieldOrientatedDriveCommand(
                        driverController.stick::getXAxis, driverController.stick::getYAxis,
                        driverController.stick::getZAxis, driveSubsystem
                )
        );
        driveCommandChooser.addOption(
                "Robot Orientated",
                new RobotOrientatedDriveCommand(
                        driverController.stick::getXAxis, driverController.stick::getYAxis,
                        driverController.stick::getZAxis, driveSubsystem
                )
        );

        ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrainRaw");
        driveTab.add("Drive Style", driveCommandChooser);
        driveTab.add(
                "Evaluate Drive Style", new NameableInstantRunWhenDisabledCommand("Evaluate", this::evaluateDriveStyle)
        );

        evaluateDriveStyle();

        driverController.buttonOne.whenPressed(driveSubsystem::zeroHeading);
        driverController.buttonTwo.whenHeld(new SetModuleRotationCommand(0.0, driveSubsystem));
        driverController.buttonThree.whileHeld(new HoldDrivePositionCommand(driveSubsystem));
    }

    private void evaluateDriveStyle() {
        Command newCommand = driveCommandChooser.getSelected();
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
