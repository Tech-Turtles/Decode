package org.firstinspires.ftc.teamcode.utility.command;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public abstract class CommandGroupBase implements Command {

    protected final List<Command> commands;
    protected final Set<Subsystem> requirements = new HashSet<>();

    public CommandGroupBase(List<Command> commands) {
        this.commands = commands;

        // Merge requirements from child commands
        for (Command cmd : commands) {
            requirements.addAll(cmd.getRequirements());
        }
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }
}