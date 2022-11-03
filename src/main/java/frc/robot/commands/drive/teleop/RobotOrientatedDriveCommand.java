package frc.robot.commands.drive.teleop;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.function.DoubleSupplier;

public class RobotOrientatedDriveCommand extends SwerveDriveCommand {

    public RobotOrientatedDriveCommand(
            DoubleSupplier xAxisSupplier, DoubleSupplier yAxisSupplier, DoubleSupplier rotationSupplier,
            SwerveDriveSubsystem driveSubsystem
    ) {
        super(xAxisSupplier, yAxisSupplier, rotationSupplier, driveSubsystem);
    }

    @Override
    public void execute() {
        double[] inputs = getNormalizedScaledRateLimitedXYTheta();
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(inputs[0], inputs[1], inputs[2]);

        setDriveChassisSpeedsWithDeadZone(chassisSpeeds);
    }
}
