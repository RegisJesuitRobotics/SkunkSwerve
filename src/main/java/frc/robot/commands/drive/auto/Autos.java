package frc.robot.commands.drive.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.commands.drive.SteerTestingCommand;
import frc.robot.commands.drive.characterize.DynamicCharacterizeDriveCommand;
import frc.robot.commands.drive.characterize.QuasistaticCharacterizeDriveCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.HashMap;
import java.util.Map;

public class Autos {
    private final HashMap<String, Command> autoMap = new HashMap<>();
    private final RaiderSwerveAutoBuilder autoBuilder;

    public Autos(SwerveDriveSubsystem driveSubsystem) {
        HashMap<String, Command> eventMap = new HashMap<>(
                Map.ofEntries(Map.entry("LockModules", new LockModulesCommand(driveSubsystem)))
        );

        autoBuilder = new RaiderSwerveAutoBuilder(eventMap, driveSubsystem);
        addPPAuto("FigureEights");
        addPPAuto("FigureEightsWithRotation");
        addPPAuto("FUN");
        addPPAuto("NoRotation");
        addPPAuto("StopPointTest");
        addPPAuto("StraightNoRotation");
        addPPAuto("StraightWithRotation");
        addPPAuto("WithRotation");
        addAuto("QuasistaticCharacterizationForward", new QuasistaticCharacterizeDriveCommand(0.4, driveSubsystem));
        addAuto("DynamicCharacterizationForward", new DynamicCharacterizeDriveCommand(8.0, driveSubsystem));
        addAuto("QuasistaticCharacterizationBackward", new QuasistaticCharacterizeDriveCommand(-0.4, driveSubsystem));
        addAuto("DynamicCharacterizationBackward", new DynamicCharacterizeDriveCommand(-8.0, driveSubsystem));
        addAuto("SteerTesting", new SteerTestingCommand(driveSubsystem));
    }

    private void addPPAuto(String name) {
        addAuto(name, autoBuilder.fullAuto(PathPlanner.loadPathGroup(name, AutoConstants.PATH_CONSTRAINTS)));
    }

    private void addAuto(String name, Command command) {
        autoMap.put(name, command);
    }

    public Map<String, Command> getAutos() {
        return autoMap;
    }
}
