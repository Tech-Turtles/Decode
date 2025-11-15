package org.firstinspires.ftc.teamcode.opmode.autonomous;


import static org.firstinspires.ftc.teamcode.utility.Constants.blueLLAngleOffset;
import static org.firstinspires.ftc.teamcode.utility.Constants.flipperAdd;
import static org.firstinspires.ftc.teamcode.utility.Constants.kicked;
import static org.firstinspires.ftc.teamcode.utility.Constants.kicking;
import static org.firstinspires.ftc.teamcode.utility.Constants.redLLAngleOffset;
import static org.firstinspires.ftc.teamcode.utility.Constants.robotHalfLength;
import static org.firstinspires.ftc.teamcode.utility.Constants.robotHalfWidth;
import static org.firstinspires.ftc.teamcode.utility.Constants.llD;
import static org.firstinspires.ftc.teamcode.utility.Constants.llI;
import static org.firstinspires.ftc.teamcode.utility.Constants.llP;
import static org.firstinspires.ftc.teamcode.utility.Constants.lowTriangle;
import static org.firstinspires.ftc.teamcode.utility.Constants.targetX;
import static org.firstinspires.ftc.teamcode.utility.Constants.targetY;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.Constants;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimer;
import org.firstinspires.ftc.teamcode.utility.PIDController;

@Config
public class OdoAuto extends RobotHardware {

    private Canvas canvas;
    private Action autonomous;
    private double shooterSetPoint = 0.0;
    private boolean shouldRun;

    private static double previousTime = 0;

    private final double kV = 0.000169;
    private final double shooterP = 0.003, shooterI = 0, shooterD = 0;
    protected PIDController shooterPID = new PIDController(shooterP, shooterI, shooterD);
    // Set point is RPM
    private final double tolerance = 20;
    public static double kStatic = 0.06;

    public static double gateOpen = 0.68;

    public static double gateClosed = 1;

    public static double gateOpenDurationSeconds = 0.3;
    protected PIDController llAnglePID = new PIDController(llP, llI, llD);
    private final ElapsedTimer gateTimer = new ElapsedTimer();
    public static double startX = -72 + robotHalfLength;
    public static double startY = 0 - robotHalfWidth - (24 - robotHalfWidth);

    @Override
    public void init() {
        super.init();
        gate.setPosition(gateClosed);
        Pose2d moveForwardEnd;
        TrajectoryActionBuilder moveForwardFinal;
        TrajectoryActionBuilder moveForward;

        if (alliance == Constants.Alliance.RED){
            drive.localizer.setPose(new Pose2d(startX, startY, Math.toRadians(180)));
            moveForwardEnd = new Pose2d(startX + 5+20, startY, Math.toRadians(180));

            moveForwardFinal =
                    drive.actionBuilder(new Pose2d(startX, startY, Math.toRadians(180))).lineToX(startX + 20);

            moveForward =
                    drive.actionBuilder(new Pose2d(startX + 5, startY, Math.toRadians(180))).lineToX(startX + 20);
        } else {
            drive.localizer.setPose(new Pose2d(startX, -startY, Math.toRadians(180)));
            moveForwardEnd = new Pose2d(startX + 5+20, -startY, Math.toRadians(90 - 67));

            moveForwardFinal =
                    drive.actionBuilder(new Pose2d(startX, -startY, Math.toRadians(90 - 67))).lineToX(startX + 20);

            moveForward =
                    drive.actionBuilder(new Pose2d(startX + 5, -startY, Math.toRadians(90 - 67))).lineToX(startX + 20);

        }
        SequentialAction shootSequence = new SequentialAction(
                new InstantAction(() ->
                {
                    gate.setPosition(gateOpen);
                }),
                new SleepAction(gateOpenDurationSeconds),
                new InstantAction(() ->
                {
                    gate.setPosition(gateClosed);
                }),
                new SleepAction(flipperAdd),
                new InstantAction(() ->
                {
                    kickerLeft.setPosition(kicked);
                    kickerRight.setPosition(kicked);
                }),
                new SleepAction(flipperAdd),
                new InstantAction(() ->
                {
                    kickerLeft.setPosition(kicked);
                    kickerRight.setPosition(kicking);
                }),
                new SleepAction(1)
        );

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        autonomous = new SequentialAction(
                moveForward.build(),
                drive.actionBuilder(moveForwardEnd).turnTo(OdoAim()).build(),
                new InstantAction(() -> //does on start, Shoot 3 balls
                {
                    shooterSetPoint = lowTriangle;
                }),
                new SleepAction(1.5 + 0.5),
                shootSequence,
                shootSequence,
                shootSequence,
                new InstantAction(() ->
                {
                    shooterSetPoint = 0;
                }),
                moveForwardFinal.build()
        );

        canvas = new Canvas();
        autonomous.preview(canvas);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        shouldRun = true;

        shooterPID.setPID(shooterP, shooterI, shooterD);
        shooterPID.setTolerance(tolerance);
    }

    @Override
    public void loop() {
        super.loop();

        // Calculate shooter rpm; ticks per second to rpm
        // 6000 rpm motor is 28 ticks per rotation
        double shooterRPM = shooterTop.getVelocity() / 28.0 * 60.0;
        double power = shooterPID.calculate(shooterRPM, shooterSetPoint);
        double combined = Math.min(1, Math.max(-1, shooterSetPoint * kV + power));

        displayData("Shooter RPM", shooterRPM);
        displayData("Setpoint RPM", shooterSetPoint);
        displayData("PID Power", power);

        if (shouldRun) {
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

            shouldRun = autonomous.run(packet);

            shooterTop.setPower(combined + Math.signum(power) * kStatic);
            shooterBottom.setPower(combined + Math.signum(power) * kStatic);


        } else {
            shooterTop.setPower(0.0);
            shooterBottom.setPower(0.0);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        }
    }


    public double OdoAim() {
        Pose2d robot = drive.localizer.getPose();
        Pose2d target;
        double offset;
        if (alliance == Constants.Alliance.RED)
        {
            target = new Pose2d((targetX), -targetY, 45);
            offset = redLLAngleOffset;
        }
        else
        {
            target = new Pose2d(-(targetX), -(targetY), 0);
            offset = blueLLAngleOffset;
        }
        double deltaX = robot.position.x - target.position.x;
        double deltaY = robot.position.y - target.position.y;
        double headingToTarget = Math.atan2(deltaY, deltaX) + Math.toRadians(offset);
        while (headingToTarget > Math.PI)
            headingToTarget -= Math.PI;
        while (headingToTarget < -Math.PI)
            headingToTarget += Math.PI;

        return headingToTarget;
    }
}

