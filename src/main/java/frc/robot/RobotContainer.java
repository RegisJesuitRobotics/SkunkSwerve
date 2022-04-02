// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.drive.FieldOrientatedDriveCommand;
import frc.robot.commands.drive.FollowPathCommand;
import frc.robot.commands.drive.SetModuleRotationCommand;
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
    // The robot's subsystems and commands are defined here...
    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();

    private final ThrustMaster driverController = new ThrustMaster(0);

    private final SendableChooser<Command> driveCommandChooser = new SendableChooser<>();
    private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        autoCommandChooser.setDefaultOption("Nothing", new InstantCommand());
        autoCommandChooser
                .addOption("No Rotation Auto",
                        new FollowPathCommand(
                                PathPlanner.loadPath("NoRotation", DriveTrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                                        DriveTrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
                                swerveDriveSubsystem));
        autoCommandChooser
                .addOption("Rotation Auto",
                        new FollowPathCommand(
                                PathPlanner.loadPath("WithRotation", DriveTrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                                        DriveTrainConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
                                swerveDriveSubsystem));
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driveCommandChooser.setDefaultOption("Field Orientated",
                new FieldOrientatedDriveCommand(driverController, swerveDriveSubsystem));
        driveCommandChooser.addOption("Robot Orientated",
                new FieldOrientatedDriveCommand(driverController, swerveDriveSubsystem));

        Shuffleboard.getTab("DriveTrainRaw").add("Drive Style", driveCommandChooser);
        Shuffleboard.getTab("DriveTrainRaw").add("Evaluate Drive Style", new InstantRunWhenDisabledCommand(
                () -> swerveDriveSubsystem.setDefaultCommand(driveCommandChooser.getSelected())));
        swerveDriveSubsystem.setDefaultCommand(driveCommandChooser.getSelected());

        driverController.buttonOne.whenPressed(swerveDriveSubsystem::zeroGyro);
        driverController.buttonTwo
                .whenHeld(new SetModuleRotationCommand(new double[]{ 0.0, 0.0, 0.0, 0.0 }, swerveDriveSubsystem));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoCommandChooser.getSelected();
    }
}
