package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class SetModuleRotationCommand extends CommandBase {
    private final SwerveModuleState[] desiredStates = new SwerveModuleState[4];
    private final SwerveDriveSubsystem driveSubsystem;

    public SetModuleRotationCommand(double allRotationDegrees, SwerveDriveSubsystem driveSubsystem) {
        this(new double[]{allRotationDegrees, allRotationDegrees, allRotationDegrees, allRotationDegrees}, driveSubsystem);
    }

    public SetModuleRotationCommand(double[] rotationDegrees, SwerveDriveSubsystem driveSubsystem) {
        if (rotationDegrees.length != 4) {
            throw new IllegalArgumentException("You must have 4 rotation degrees");
        }

        for (int i = 0; i < 4; i++) {
            desiredStates[i] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(rotationDegrees[i]));
        }
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        driveSubsystem.setRawStates(desiredStates);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.atDesiredStates();
    }
}
