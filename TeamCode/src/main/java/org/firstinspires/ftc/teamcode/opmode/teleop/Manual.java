package org.firstinspires.ftc.teamcode.opmode.teleop;

import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utility.command.Command;

public class Manual extends RobotHardware {

    DriveSubsystem drive;

    @Override
    public void init() {
        super.init();

        drive = new DriveSubsystem(batteryVoltageSensor, localizer);

        scheduler.registerSubsystem(drive);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
    }

    public class FieldCentricDriveCommand implements Command {
        private final DriveSubsystem drive;

        public FieldCentricDriveCommand(DriveSubsystem drive) {
            this.drive = drive;
            addRe(drive);
        }

        @Override
        public void execute() {
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rot = gamepad1.right_stick_x;

            drive.setDrivePowersField(x, y, rot, 0);
        }

        @Override
        public void end(boolean interrupted) {
            drive.stop();
        }
    }
}
