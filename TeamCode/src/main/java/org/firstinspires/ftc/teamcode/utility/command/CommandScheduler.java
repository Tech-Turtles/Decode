package org.firstinspires.ftc.teamcode.utility.command;

import java.util.*;

public class CommandScheduler {

    private static final CommandScheduler instance = new CommandScheduler();

    public static CommandScheduler getInstance() {
        return instance;
    }

    private final Set<Command> scheduledCommands = new HashSet<>();
    private final Map<Subsystem, Command> requirements = new HashMap<>();
    private final Set<Subsystem> subsystems = new HashSet<>();

    public void registerSubsystem(Subsystem subsystem) {
        subsystems.add(subsystem);
    }

    public void schedule(Command command) {
        // Check for conflicts & cancel conflicting commands
        for (Subsystem requirement : command.getRequirements()) {
            if (requirements.containsKey(requirement)) {
                cancel(requirements.get(requirement));
            }
        }

        // Claim subsystem ownership
        for (Subsystem requirement : command.getRequirements()) {
            requirements.put(requirement, command);
        }

        scheduledCommands.add(command);
        command.initialize();
    }

    public void cancel(Command command) {
        if (scheduledCommands.remove(command)) {
            command.end(true); // interrupted
            requirements.values().removeIf(c -> c == command);
        }
    }

    public void cancelAll() {
        for (Command cmd : new HashSet<>(scheduledCommands)) {
            cancel(cmd);
        }
    }
    public void run() {
        // Run periodic
        for (Subsystem subsystem : subsystems) {
            subsystem.periodic();
        }

        // Execute scheduled commands
        for (Command command : new ArrayList<>(scheduledCommands)) {
            command.execute();

            if (command.isFinished()) {
                command.end(false);
                scheduledCommands.remove(command);

                // Free the subsystem requirements
                requirements.values().removeIf(c -> c == command);
            }
        }

        // Schedule default commands if nothing is running on a subsystem
        for (Subsystem subsystem : subsystems) {
            if (!requirements.containsKey(subsystem)) {
                Command defaultCmd = subsystem.getDefaultCommand();
                if (defaultCmd != null && !scheduledCommands.contains(defaultCmd)) {
                    schedule(defaultCmd);
                }
            }
        }
    }
}