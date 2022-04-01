package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class InstantRunWhenDisabledCommand extends InstantCommand {
    public InstantRunWhenDisabledCommand(Runnable runnable) {
        super(runnable);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
