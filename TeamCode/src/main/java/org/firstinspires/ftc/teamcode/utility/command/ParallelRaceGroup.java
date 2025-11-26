package org.firstinspires.ftc.teamcode.utility.command;

import java.util.*;

public class ParallelRaceGroup extends CommandGroupBase {

    private final Set<Command> runningCommands = new HashSet<>();
    private boolean done = false;

    public ParallelRaceGroup(Command... commands) {
        super(List.of(commands));
    }

    @Override
    public void initialize() {
        runningCommands.addAll(commands);
        commands.forEach(Command::initialize);
    }

    @Override
    public void execute() {
        for (Command cmd : new HashSet<>(runningCommands)) {
            cmd.execute();
            if (cmd.isFinished()) {
                cmd.end(false);
                done = true;
                runningCommands.remove(cmd);
                break;
            }
        }

        if (done) {
            // Interrupt all others
            for (Command remaining : runningCommands) {
                remaining.end(true);
            }
            runningCommands.clear();
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}