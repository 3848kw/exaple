package frc.robot.logging;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public final class LoggedCommandScheduler {
    private static final String LOG_KEY = "Commands";
    private static final String ALERT_TYPE = "Alerts";

    private static final Set<Command> runningNonInterrupters = new HashSet<>();
    private static final Map<Command, Command> runningInterrupters = new HashMap<>();
    private static final Map<Subsystem, Command> requiredSubsystems = new HashMap<>();

    private static final NetworkTable root = NetworkTableInstance.getDefault().getTable(LOG_KEY);
    private static final NetworkTable runningTbl   = root.getSubTable("Running");
    private static final NetworkTable subsystemsTbl = root.getSubTable("Subsystems");

    private LoggedCommandScheduler() {}

    private static void commandStarted(final Command command) {
        if (!runningInterrupters.containsKey(command)) {
            runningNonInterrupters.add(command);
        }
        for (final Subsystem subsystem : command.getRequirements()) {
            requiredSubsystems.put(subsystem, command);
        }
    }

    private static void commandEnded(final Command command) {
        runningNonInterrupters.remove(command);
        runningInterrupters.remove(command);
        for (final Subsystem subsystem : command.getRequirements()) {
            requiredSubsystems.remove(subsystem);
        }
    }

    public static void init(final CommandScheduler scheduler) {
        scheduler.onCommandInitialize(LoggedCommandScheduler::commandStarted);
        scheduler.onCommandFinish(LoggedCommandScheduler::commandEnded);
        scheduler.onCommandInterrupt((interrupted, interrupting) -> {
            interrupting.ifPresent(interrupter -> runningInterrupters.put(interrupter, interrupted));
            commandEnded(interrupted);
        });

        runningTbl.getEntry(".type").setString(ALERT_TYPE);
        subsystemsTbl.getEntry(".type").setString(ALERT_TYPE);
    }

    private static void logRunningCommands() {
        runningTbl.getEntry(".type").setString(ALERT_TYPE);

        final String[] warnings = runningNonInterrupters.stream()
                .map(Command::getName)
                .toArray(String[]::new);
        runningTbl.getEntry("warnings").setStringArray(warnings);

        final String[] errors = runningInterrupters.entrySet().stream()
                .map(entry -> {
                    final Command interrupter = entry.getKey();
                    final Command interrupted = entry.getValue();
                    final Set<Subsystem> common = new HashSet<>(interrupter.getRequirements());
                    common.retainAll(interrupted.getRequirements());
                    final String commonNames = String.join(",",
                            common.stream().map(Subsystem::getName).toArray(String[]::new));
                    return interrupter.getName()
                            + " interrupted "
                            + interrupted.getName()
                            + (common.isEmpty() ? "" : " (" + commonNames + ")");
                })
                .toArray(String[]::new);
        runningTbl.getEntry("errors").setStringArray(errors);

        runningTbl.getEntry("infos").setStringArray(new String[0]);
    }

    private static void logRequiredSubsystems() {
        subsystemsTbl.getEntry(".type").setString(ALERT_TYPE);

        final String[] infos = requiredSubsystems.entrySet().stream()
                .map(e -> e.getKey().getName() + " (" + e.getValue().getName() + ")")
                .toArray(String[]::new);
        subsystemsTbl.getEntry("infos").setStringArray(infos);

        subsystemsTbl.getEntry("warnings").setStringArray(new String[0]);
        subsystemsTbl.getEntry("errors").setStringArray(new String[0]);
    }

    public static void periodic() {
        logRunningCommands();
        logRequiredSubsystems();
    }
}