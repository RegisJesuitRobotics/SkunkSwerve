package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.telemetry.CommandSchedulerLogger;
import frc.robot.telemetry.MiscRobotTelemetryAndAlerts;
import frc.robot.telemetry.TelemetryPowerDistribution;



/**
 * The VM is configured to automatically run this class, and to call the methods
 * corresponding to each mode, as described in the TimedRobot documentation. If
 * you change the name of this class or the package after creating this project,
 * you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    private TelemetryPowerDistribution telemetryPowerDistribution;
    private MiscRobotTelemetryAndAlerts miscRobotTelemetryAndAlerts;

    /**
     * This method is run when the robot is first started up and should be used for
     * any initialization code.
     */
    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
        DriverStation.silenceJoystickConnectionWarning(true);
        // No reason to log them as everything on NT is already logged
        DataLogManager.logNetworkTables(false);
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        CommandSchedulerLogger.getInstance().start();

        telemetryPowerDistribution = new TelemetryPowerDistribution(0, ModuleType.kCTRE);
        miscRobotTelemetryAndAlerts = new MiscRobotTelemetryAndAlerts();

        robotContainer = new RobotContainer();
    }


    /**
     * This method is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        miscRobotTelemetryAndAlerts.logValues();
        telemetryPowerDistribution.logValues();
    }


    /** This method is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}


    @Override
    public void disabledPeriodic() {}


    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }


    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}


    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }


    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}


    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }


    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
