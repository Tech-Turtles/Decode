package org.firstinspires.ftc.teamcode.utility.command;

import java.util.*;

public class ParallelCommandGroup extends CommandGroupBase {

    private final Set<Command> runningCommands = new HashSet<>();

    public ParallelCommandGroup(Command... commands) {
        super(List.of(commands));
    }

    @Override
    public void initialize() {
        runningCommands.addAll(commands);
        commands.forEach(Command::initialize);
    }

    @Override
    public void execute() {
        for (Iterator<Command> it = runningCommands.iterator(); it.hasNext();) {
            Command cmd = it.next();
            cmd.execute();

            if (cmd.isFinished()) {
                cmd.end(false);
                it.remove();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return runningCommands.isEmpty();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            runningCommands.forEach(c -> c.end(true));
        }
    }
}
