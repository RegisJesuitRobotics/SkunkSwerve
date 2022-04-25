// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drive.*;
import frc.robot.commands.util.InstantRunWhenDisabledCommand;
import frc.robot.joysticks.ThrustMaster;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;



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

    public RobotContainer() {
        configureButtonBindings();
        configureAutos();
    }

    private void configureAutos() {
        autoCommandChooser.setDefaultOption("Nothing", new InstantCommand());
        autoCommandChooser.addOption("UpDownWithRotation", new FollowPathCommand("WithRotation", driveSubsystem));
        autoCommandChooser.addOption("UpDownNoRotation", new FollowPathCommand("NoRotation", driveSubsystem));
        autoCommandChooser
                .addOption("StraightWithRotation", new FollowPathCommand("StraightWithRotation", driveSubsystem));
        autoCommandChooser.addOption("StraightNoRotation", new FollowPathCommand("StraightNoRotation", driveSubsystem));
        autoCommandChooser.addOption("FigureEights", new FollowPathCommand("FigureEights", driveSubsystem));
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
        driveTab.add("Evaluate Drive Style", new InstantRunWhenDisabledCommand(this::evaluateDriveStyle));
        driveTab.add("Reset to Absolute", new InstantRunWhenDisabledCommand(driveSubsystem::setAllModulesToAbsolute));

        evaluateDriveStyle();

        driverController.buttonOne.whenPressed(driveSubsystem::zeroHeading);
        driverController.buttonTwo.whenHeld(new SetModuleRotationCommand(0.0, driveSubsystem));
        driverController.buttonThree.whileHeld(new HoldDrivePositionCommand(driveSubsystem));
    }

    private void evaluateDriveStyle() {
        Command newCommand = driveCommandChooser.getSelected();
        Command oldCommand = driveSubsystem.getDefaultCommand();

        if (newCommand.equals(oldCommand)) {
            return;
        }

        driveSubsystem.setDefaultCommand(newCommand);
        if (oldCommand != null) {
            oldCommand.cancel();
        }
    }

    public Command getAutonomousCommand() {
        return autoCommandChooser.getSelected();
    }
}
