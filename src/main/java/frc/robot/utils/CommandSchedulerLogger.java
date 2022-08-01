package frc.robot.utils;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class CommandSchedulerLogger {
    private static final String tableName = "/commandScheduler/";

    private final StringLogEntry initializedCommandsEntry = new StringLogEntry(
            DataLogManager.getLog(), tableName + "initialized"
    );
    private final StringLogEntry finishedCommandsEntry = new StringLogEntry(
            DataLogManager.getLog(), tableName + "finished"
    );

    public CommandSchedulerLogger(CommandScheduler commandScheduler) {
        commandScheduler.onCommandInitialize(this::onCommandInitializeConsumer);
        commandScheduler.onCommandFinish(this::onCommandFinishConsumer);
    }

    private void onCommandInitializeConsumer(Command command) {
        initializedCommandsEntry.append(command.getName() + " (" + command.hashCode() + ")");
    }

    private void onCommandFinishConsumer(Command command) {
        finishedCommandsEntry.append(command.getName() + " (" + command.hashCode() + ")");
    }
}
