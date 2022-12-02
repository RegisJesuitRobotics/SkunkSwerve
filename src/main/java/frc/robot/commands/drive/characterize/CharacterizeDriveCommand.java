package frc.robot.commands.drive.characterize;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        List<Double> data = new ArrayList<>();
        for (int i = 0; i < velocityList.size(); i++) {
            data.add(timeList.get(i));
            data.add(voltageList.get(i));
            data.add(positionList.get(i));
            data.add(velocityList.get(i));
        }
        SmartDashboard.putString(
                "SysIdTelemetry", getTestName() + ";" + data.toString().substring(1, data.toString().length() - 1)
        );
        SmartDashboard.putNumber("SysIdAckNumber", 1);

        driveSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
