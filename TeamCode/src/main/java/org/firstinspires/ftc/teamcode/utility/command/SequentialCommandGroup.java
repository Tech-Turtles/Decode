package org.firstinspires.ftc.teamcode.utility.command;

import java.util.List;

public class SequentialCommandGroup extends CommandGroupBase {

    private int index = 0;

    public SequentialCommandGroup(Command... commands) {
        super(List.of(commands));
    }

    @Override
    public void initialize() {
        if (!commands.isEmpty()) {
            commands.get(0).initialize();
        }
    }

    @Override
    public void execute() {
        if (index >= commands.size()) return;

        Command current = commands.get(index);
        current.execute();

        if (current.isFinished()) {
            current.end(false);
            index++;

            if (index < commands.size()) {
                commands.get(index).initialize();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && index < commands.size()) {
            commands.get(index).end(true);
        }
    }

    @Override
    public boolean isFinished() {
        return index >= commands.size();
    }
}