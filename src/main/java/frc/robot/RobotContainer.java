// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.drive.FieldOrientatedDriveCommand;
import frc.robot.commands.drive.FollowPathCommand;
import frc.robot.commands.drive.RobotOrientatedDriveCommand;
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
    private final SwerveDriveSubsystem driveSubsystem = new SwerveDriveSubsystem();

    private final ThrustMaster driverController = new ThrustMaster(0);
//    private final PlaystationController driverController = new PlaystationController(0);

    private final SendableChooser<Command> driveCommandChooser = new SendableChooser<>();
    private final SendableChooser<Command> autoCommandChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();
        configureAutos();
    }

    private void configureAutos() {
        autoCommandChooser.setDefaultOption("Nothing", new InstantCommand());
        autoCommandChooser.addOption("UpDownWithRotation", new FollowPathCommand("WithRotation", driveSubsystem));
        autoCommandChooser.addOption("UpDownNoRotation", new FollowPathCommand("NoRotation", driveSubsystem));
        autoCommandChooser.addOption("StraightWithRotation",
                new FollowPathCommand("StraightWithRotation", driveSubsystem));
        autoCommandChooser.addOption("StraightNoRotation", new FollowPathCommand("StraightNoRotation", driveSubsystem));
        autoCommandChooser.addOption("FigureEights", new FollowPathCommand("FigureEights", driveSubsystem));
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driveCommandChooser.setDefaultOption("Field Orientated",
                new FieldOrientatedDriveCommand(driverController.stick::getXAxis, driverController.stick::getYAxis,
                        driverController.stick::getZAxis, driveSubsystem));
        driveCommandChooser.addOption("Robot Orientated",
                new RobotOrientatedDriveCommand(driverController.stick::getXAxis, driverController.stick::getYAxis,
                        driverController.stick::getZAxis, driveSubsystem));
//        driveCommandChooser.setDefaultOption("Field Orientated",
//                new FieldOrientatedDriveCommand(driverController.leftThumb::getXAxis, driverController.leftThumb::getYAxis,
//                        driverController.rightThumb::getYAxis, swerveDriveSubsystem));
//        driveCommandChooser.addOption("Robot Orientated",
//                new RobotOrientatedDriveCommand(driverController.leftThumb::getXAxis, driverController.leftThumb::getYAxis,
//                        driverController.rightThumb::getXAxis, swerveDriveSubsystem));

        Shuffleboard.getTab("DriveTrainRaw").add("Drive Style", driveCommandChooser);
        Shuffleboard.getTab("DriveTrainRaw").add("Evaluate Drive Style", new InstantRunWhenDisabledCommand(
                () -> driveSubsystem.setDefaultCommand(driveCommandChooser.getSelected())));
        driveSubsystem.setDefaultCommand(driveCommandChooser.getSelected());

        driverController.buttonOne.whenPressed(driveSubsystem::zeroGyro);
        driverController.buttonTwo.whenHeld(new SetModuleRotationCommand(0.0, driveSubsystem));
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
