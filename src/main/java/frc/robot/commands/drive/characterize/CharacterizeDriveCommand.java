package frc.robot.commands.drive.characterize;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.ArrayList;
import java.util.List;

public abstract class CharacterizeDriveCommand extends CommandBase {
    private final SwerveDriveSubsystem driveSubsystem;
    protected final Timer timer = new Timer();

    protected double lastVoltage = 0.0;

    protected final List<Double> timeList = new ArrayList<>();
    protected final List<Double> voltageList = new ArrayList<>();
    protected final List<Double> velocityList = new ArrayList<>();

    public CharacterizeDriveCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    protected abstract double getVoltage();

    private void logValues() {
        timeList.add(timer.get());
        voltageList.add(lastVoltage);
        velocityList.add(driveSubsystem.getAverageDriveVelocityMetersSecond());
    }

    @Override
    public void initialize() {
        voltageList.clear();
        velocityList.clear();
        timeList.clear();

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        logValues();

        lastVoltage = getVoltage();
        driveSubsystem.setCharacterizationVoltage(lastVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        NetworkTableEntry voltageListEntry = NetworkTableInstance.getDefault().getTable("Characterize")
                .getEntry("voltageList");
        NetworkTableEntry velocityListEntry = NetworkTableInstance.getDefault().getTable("Characterize")
                .getEntry("velocityList");
        NetworkTableEntry timeListEntry = NetworkTableInstance.getDefault().getTable("Characterize")
                .getEntry("timeList");
        voltageListEntry.setDoubleArray(voltageList.stream().mapToDouble(Double::doubleValue).toArray());
        velocityListEntry.setDoubleArray(velocityList.stream().mapToDouble(Double::doubleValue).toArray());
        timeListEntry.setDoubleArray(timeList.stream().mapToDouble(Double::doubleValue).toArray());

        driveSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
