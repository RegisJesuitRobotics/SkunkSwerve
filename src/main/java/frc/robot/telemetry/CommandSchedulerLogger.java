package frc.robot.telemetry;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.telemetry.types.StringTelemetryEntry;

public class CommandSchedulerLogger {
    private static CommandSchedulerLogger instance;
    private static final String tableName = "/commandScheduler/";

    public static CommandSchedulerLogger getInstance() {
        if (instance == null) {
            instance = new CommandSchedulerLogger(CommandScheduler.getInstance());
        }
        return instance;
    }

    private final StringTelemetryEntry initializedCommandsEntry = new StringTelemetryEntry(
            tableName + "initialized", false, false
    );
    private final StringTelemetryEntry finishedCommandsEntry = new StringTelemetryEntry(
            tableName + "finished", false, false
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
