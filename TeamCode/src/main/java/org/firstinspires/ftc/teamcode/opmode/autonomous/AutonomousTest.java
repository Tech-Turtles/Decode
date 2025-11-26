package org.firstinspires.ftc.teamcode.opmode.autonomous;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utility.command.Command;
import org.firstinspires.ftc.teamcode.utility.command.InstantCommand;
import org.firstinspires.ftc.teamcode.utility.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.utility.command.WaitCommand;

import java.util.function.Supplier;

@Config
@Autonomous
public class AutonomousTest extends RobotHardware {

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
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPose(startPose);

        Action path = drive.actionBuilder(startPose)
                .lineToX(30)
                .turn(Math.toRadians(90))
                .build();

        Command auto = new SequentialCommandGroup(
                new InstantCommand(() -> drive.setPose(startPose)),
                new RRActionCommand(drive, path),
                new WaitCommand(5),
                new RRActionCommand(drive, () -> {
                    Pose2d current = drive.localizer.getPose();

                    return drive.actionBuilder(current)
                            .splineTo(new Vector2d(current.position.x+10, current.position.y-10), 0)
                            .build();
                })
        );

        scheduler.schedule(auto);
    }

    public class RRActionCommand extends Command {

        private final DriveSubsystem drive;
        private Action action;
        private boolean isComplete = false;

        // Optional supplier for dynamic building
        private Supplier<Action> actionSupplier;

        public RRActionCommand(DriveSubsystem drive, Action action) {
            this.drive = drive;
            this.action = action;
            addRequirements(drive);
        }

        public RRActionCommand(DriveSubsystem drive, Supplier<Action> actionSupplier) {
            this.drive = drive;
            this.actionSupplier = actionSupplier;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            if (actionSupplier != null) {
                action = actionSupplier.get();
            }

            isComplete = false;
        }

        @Override
        public void execute() {
            if (isComplete || action == null) return;

            boolean stillRunning = action.run(new TelemetryPacket());

            if (!stillRunning) {
                isComplete = true;
            }
        }

        @Override
        public boolean isFinished() {
            return isComplete;
        }
    }
}
