package frc.robot.commands.drive.characterize;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.ArrayList;
import java.util.List;

public abstract class CharacterizeDriveCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;
    protected final Timer timer = new Timer();

    protected final List<Double> timeList = new ArrayList<>();
    protected final List<Double> voltageList = new ArrayList<>();
    protected final List<Double> velocityList = new ArrayList<>();
    protected final List<Double> positionList = new ArrayList<>();

    private final StringPublisher testNamePublisher = NetworkTableInstance.getDefault().getStringTopic("char/testName")
            .publish();
    private final DoubleArrayPublisher timePublisher = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("char/time").publish();
    private final DoubleArrayPublisher voltagePublisher = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("char/voltage").publish();
    private final DoubleArrayPublisher velocityPublisher = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("char/velocity").publish();
    private final DoubleArrayPublisher positionPublisher = NetworkTableInstance.getDefault()
            .getDoubleArrayTopic("char/position").publish();

    public CharacterizeDriveCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    protected abstract double getVoltage();

    protected abstract String getTestName();

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
        double voltage = getVoltage();

        timeList.add(timer.get());
        voltageList.add(voltage);
        velocityList.add(driveSubsystem.getAverageDriveVelocityMetersSecond());
        positionList.add(driveSubsystem.getAverageDrivePositionMeters());

        driveSubsystem.setCharacterizationVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        testNamePublisher.set(getTestName());
        timePublisher.set(timeList.stream().mapToDouble(Double::doubleValue).toArray());
        voltagePublisher.set(voltageList.stream().mapToDouble(Double::doubleValue).toArray());
        velocityPublisher.set(velocityList.stream().mapToDouble(Double::doubleValue).toArray());
        positionPublisher.set(positionList.stream().mapToDouble(Double::doubleValue).toArray());

        driveSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
