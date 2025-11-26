package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utility.command.Command;

@TeleOp
@Config
public class Manual extends RobotHardware {

    DriveSubsystem drive;

    @Override
    public void init() {
        super.init();

        drive = new DriveSubsystem(batteryVoltageSensor, localizer);

        scheduler.registerSubsystem(drive);

        drive.setDefaultCommand(new FieldCentricDriveCommand(drive));
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

    public class FieldCentricDriveCommand extends Command {
        private final DriveSubsystem drive;

        public FieldCentricDriveCommand(DriveSubsystem drive) {
            this.drive = drive;
            addRequirements(drive);
        }

        @Override
        public void execute() {
            drive.setDrivePowersField(primary.left_stick_y, primary.left_stick_x, primary.right_stick_x, 0.0);
        }

        @Override
        public void end(boolean interrupted) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), 0));
        }
    }
}
