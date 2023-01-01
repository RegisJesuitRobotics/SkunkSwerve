package frc.robot.commands.drive.characterize;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import java.util.ArrayList;
import java.util.List;

public abstract class CharacterizeDriveCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;
    private final Timer timer = new Timer();

    private final List<Double> timeList = new ArrayList<>();
    private final List<Double> voltageList = new ArrayList<>();
    private final List<Double> velocityList = new ArrayList<>();
    private final List<Double> positionList = new ArrayList<>();

    private final DoubleArrayPublisher timePublisher =
            NetworkTableInstance.getDefault().getDoubleArrayTopic("char/time").publish();
    private final DoubleArrayPublisher voltagePublisher = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("char/voltage")
            .publish();
    private final DoubleArrayPublisher velocityPublisher = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("char/velocity")
            .publish();
    private final DoubleArrayPublisher positionPublisher = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("char/position")
            .publish();

    public CharacterizeDriveCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    protected abstract double getVoltage(double currentTime);

    @Override
    public void initialize() {
        timeList.clear();
        voltageList.clear();
        velocityList.clear();
        positionList.clear();

        driveSubsystem.resetModuleEncoderPositions();

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double voltage = getVoltage(timer.get());

        timeList.add(timer.get());
        voltageList.add(voltage);
        velocityList.add(driveSubsystem.getAverageDriveVelocityMetersSecond());
        positionList.add(driveSubsystem.getAverageDrivePositionMeters());

        driveSubsystem.setCharacterizationVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stopMovement();

        timePublisher.set(timeList.stream().mapToDouble(Double::doubleValue).toArray());
        voltagePublisher.set(
                voltageList.stream().mapToDouble(Double::doubleValue).toArray());
        velocityPublisher.set(
                velocityList.stream().mapToDouble(Double::doubleValue).toArray());
        positionPublisher.set(
                positionList.stream().mapToDouble(Double::doubleValue).toArray());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
