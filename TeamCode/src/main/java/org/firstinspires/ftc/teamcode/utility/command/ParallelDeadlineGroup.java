package org.firstinspires.ftc.teamcode.utility.command;

import java.util.*;

public class ParallelDeadlineGroup extends CommandGroupBase {

    private final Command deadline;
    private final Set<Command> others = new HashSet<>();

    public ParallelDeadlineGroup(Command deadline, Command... parallel) {
        super(build(deadline, parallel));
        this.deadline = deadline;
        others.addAll(List.of(parallel));
    }

    private static List<Command> build(Command d, Command[] p) {
        List<Command> list = new ArrayList<>();
        list.add(d);
        list.addAll(List.of(p));
        return list;
    }

    @Override
    public void initialize() {
        deadline.initialize();
        others.forEach(Command::initialize);
    }

    @Override
    public void execute() {
        deadline.execute();

        for (Command other : new HashSet<>(others)) {
            other.execute();
        }

        if (deadline.isFinished()) {
            deadline.end(false);
            others.forEach(cmd -> cmd.end(true));
            others.clear();
        }
    }

    @Override
    public boolean isFinished() {
        return deadline.isFinished();
    }
}