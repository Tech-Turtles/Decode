package org.firstinspires.ftc.teamcode.utility.command;

public abstract class Subsystem {
    private Command defaultCommand;

    public void setDefaultCommand(Command command) {
        this.defaultCommand = command;
    }

    public Command getDefaultCommand() {
        return defaultCommand;
    }

    /** Called once per loop regardless of what commands run */
    public void periodic() {}
}