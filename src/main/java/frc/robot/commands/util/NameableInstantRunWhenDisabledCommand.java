package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class NameableInstantRunWhenDisabledCommand extends InstantCommand {
    private final String name;

    /**
     * Creates a new InstantCommand that runs the given Runnable with the given
     * requirements.
     *
     * @param toRun        the Runnable to run
     * @param requirements the subsystems required by this command
     */
    public NameableInstantRunWhenDisabledCommand(String name, Runnable toRun, Subsystem... requirements) {
        super(toRun, requirements);
        this.name = name;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
