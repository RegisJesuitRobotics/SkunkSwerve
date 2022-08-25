package frc.robot.utils.logging;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class CommandSchedulerLogger {
    private static CommandSchedulerLogger instance;
    private static final String tableName = "/commandScheduler/";

    public static CommandSchedulerLogger getInstance() {
        if (instance == null) {
            instance = new CommandSchedulerLogger(CommandScheduler.getInstance());
        }
        return instance;
    }

    private final StringLogEntry initializedCommandsEntry = new StringLogEntry(
            DataLogManager.getLog(), tableName + "initialized"
    );
    private final StringLogEntry finishedCommandsEntry = new StringLogEntry(
            DataLogManager.getLog(), tableName + "finished"
    );

    private final CommandScheduler commandScheduler;

    private CommandSchedulerLogger(CommandScheduler commandScheduler) {
        this.commandScheduler = commandScheduler;
    }

    public void start() {
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
