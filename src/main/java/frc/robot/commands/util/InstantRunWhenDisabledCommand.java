package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class InstantRunWhenDisabledCommand extends InstantCommand {

    /**
     * Creates a new InstantCommand that runs the given Runnable with the given
     * requirements.
     *
     * @param toRun        the Runnable to run
     * @param requirements the subsystems required by this command
     */
    public InstantRunWhenDisabledCommand(Runnable toRun, Subsystem... requirements) {
        super(toRun, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
