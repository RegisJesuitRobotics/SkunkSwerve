package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.ArrayList;
import java.util.List;

public class CharacterizeDriveCommand extends CommandBase {
    private static final double RAMP_RATE_VOLT_SECOND = 0.05;

    private final SwerveDriveSubsystem driveSubsystem;
    private final Timer timer = new Timer();

    private double lastVoltage = 0.0;

    private final List<Double> voltageList = new ArrayList<>();
    private final List<Double> velocityList = new ArrayList<>();

    public CharacterizeDriveCommand(SwerveDriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        voltageList.clear();
        velocityList.clear();

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double lastVelocity = driveSubsystem.getAverageDriveVelocityMetersSecond();
        voltageList.add(lastVoltage);
        velocityList.add(lastVelocity);

        double currentVoltage = timer.get() * RAMP_RATE_VOLT_SECOND;
        driveSubsystem.setCharacterizationVoltage(currentVoltage);

        lastVoltage = currentVoltage;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(voltageList.size());
        System.out.println(voltageList);
        System.out.println(velocityList);

        driveSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
