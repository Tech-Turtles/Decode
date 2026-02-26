package org.firstinspires.ftc.teamcode.opmode.autonomous;


import static org.firstinspires.ftc.teamcode.utility.Constants.autoOffset;
import static org.firstinspires.ftc.teamcode.utility.Constants.autoShootTime;
import static org.firstinspires.ftc.teamcode.utility.Constants.lowTriangleDipTol;
import static org.firstinspires.ftc.teamcode.utility.Constants.robotHalfLength;
import static org.firstinspires.ftc.teamcode.utility.Constants.robotHalfWidth;
import static org.firstinspires.ftc.teamcode.utility.Constants.llD;
import static org.firstinspires.ftc.teamcode.utility.Constants.llI;
import static org.firstinspires.ftc.teamcode.utility.Constants.llP;
import static org.firstinspires.ftc.teamcode.utility.Constants.lowTriangle;
import static org.firstinspires.ftc.teamcode.utility.Constants.targetX;
import static org.firstinspires.ftc.teamcode.utility.Constants.targetY;
import static org.firstinspires.ftc.teamcode.utility.Constants.autoWait;
import static org.firstinspires.ftc.teamcode.utility.Constants.autoWaitTime;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

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
    protected PIDController llAnglePID = new PIDController(llP, llI, llD);
    private final ElapsedTimer gateTimer = new ElapsedTimer();
    public static double startX = 72 - robotHalfLength;
    public static double startY = (24 - robotHalfWidth);
    public static double moveForwardX = startX - 4;
    public static double moveForwardFinalX = moveForwardX - 27;
    public static double testAutoLowTriangle = 4200;
    public static double startWaitTime = 0.0;
    public static double shooterRPM = 0.0;
    public static int shotCount = 0;
    public static double dipTol = lowTriangleDipTol;




    @Override
    public void init() {
        super.init();
        Pose2d moveForwardEnd;
        TrajectoryActionBuilder moveForwardFinal;
        TrajectoryActionBuilder moveForward;
        double targetAngle = 180;

        if (autoWait)
            startWaitTime = autoWaitTime + 3.5;


        if (alliance == Constants.Alliance.RED){
            drive.localizer.setPose(new Pose2d(startX, startY, Math.toRadians(0)));

            moveForwardEnd = new Pose2d(moveForwardX, startY, Math.toRadians(0));
            targetAngle = OdoAim(moveForwardEnd);

            moveForwardFinal =
                    drive.actionBuilder(new Pose2d(startX, startY, Math.toRadians(0))).lineToX(moveForwardFinalX);

            moveForward =
                    drive.actionBuilder(new Pose2d(startX, startY, Math.toRadians(0))).lineToX(moveForwardX);



        } else if (alliance == Constants.Alliance.BLUE) {
            drive.localizer.setPose(new Pose2d(startX, -startY, Math.toRadians(0)));

            moveForwardEnd = new Pose2d(moveForwardX, -startY, Math.toRadians(0));
            targetAngle = OdoAim(moveForwardEnd);

            moveForwardFinal =
                    drive.actionBuilder(new Pose2d(startX, -startY, Math.toRadians(0))).lineToX(moveForwardFinalX);

            moveForward =
                    drive.actionBuilder(new Pose2d(startX, -startY, Math.toRadians(0))).lineToX(moveForwardX);

        // Test mode
        }  else {
            testAutoLowTriangle = lowTriangle - 75;

            drive.localizer.setPose(new Pose2d(startX, startY, Math.toRadians(0)));

            moveForwardEnd = new Pose2d(moveForwardX, startY, Math.toRadians(0));
            targetAngle = OdoAim(moveForwardEnd);

            moveForwardFinal =
                    drive.actionBuilder(new Pose2d(startX, startY, Math.toRadians(0))).lineToX(moveForwardX + 20);

            moveForward =
                    drive.actionBuilder(new Pose2d(startX, startY, Math.toRadians(0))).lineToX(moveForwardX);
        }







//        SequentialAction shootSequence =
//                new SequentialAction(
//                        new SequentialAction(
//                                new InstantAction(()->{
//                                    while (shotCount < 3) {
//                                        new SequentialAction(
//                                                new InstantAction(()-> {
//                                                    if (shooterSetPoint != 0 && shooterRPM >= shooterSetPoint - dipTol) {
//                                                        Transfer1.setPower(1);
//                                                        Transfer2.setPower(1);
//                                                        shotCount += 1;
//                                                    }
//                                                }),
//                                                new InstantAction(()-> {
//                                                    while (shooterRPM > shooterSetPoint - dipTol){
//                                                        Transfer1.setPower(1);
//                                                        Transfer2.setPower(1);
//                                                    };
//                                                    Transfer1.setPower(0);
//                                                    Transfer2.setPower(0);
//                                                })
//                                                );
//                                    }
//
//                                })
//                        )
//                );

//        SequentialAction shootSequence =
//                new SequentialAction(
//                        new SequentialAction(
//                                new InstantAction(() -> {
//                                    if (shooterSetPoint != 0 && shooterRPM >= shooterSetPoint - dipTol) {
//                                        Transfer1.setPower(1);
//                                        Transfer2.setPower(1);
//                                    }
//                                }),
//                                new SleepAction(autoShootSleepTime),
//                                new InstantAction(()->{
//                                    Transfer1.setPower(0);
//                                    Transfer2.setPower(0);
//                                })
//                        ),
//                        new SequentialAction(
//                                new InstantAction(() -> {
//                                    if (shooterSetPoint != 0 && shooterRPM >= shooterSetPoint - dipTol) {
//                                        Transfer1.setPower(1);
//                                        Transfer2.setPower(1);
//                                    }
//                                }),
//                                new SleepAction(autoShootSleepTime),
//                                new InstantAction(()->{
//                                    Transfer1.setPower(0);
//                                    Transfer2.setPower(0);
//                                })
//                        ),
//                        new SequentialAction(
//                                new InstantAction(() -> {
//                                    if (shooterSetPoint != 0 && shooterRPM >= shooterSetPoint - dipTol) {
//                                        Transfer1.setPower(1);
//                                        Transfer2.setPower(1);
//                                    }
//                                }),
//                                new SleepAction(autoShootSleepTime),
//                                new InstantAction(()->{
//                                    Transfer1.setPower(0);
//                                    Transfer2.setPower(0);
//                                })
//                        ),
//                        new InstantAction(() -> {
//                            shooterSetPoint = 0;
//                        })
//                );


        SequentialAction shootSequence =
                new SequentialAction(
                        new SequentialAction(
                                new InstantAction(() -> {
                                        Transfer1.setPower(1);
                                        Transfer2.setPower(1);
                                }),
                                new SleepAction(autoShootTime),
                                new InstantAction(()->{
                                    Transfer1.setPower(0);
                                    Transfer2.setPower(0);
                                })
                        ),
                        new InstantAction(() -> {
                            shooterSetPoint = 0;
                        })
                );

        SequentialAction shooterSpinUp =
                new SequentialAction(
                        new InstantAction(() -> {
                            shooterSetPoint = lowTriangle;
                        }),
                        new SleepAction(1.5)
                );


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        autonomous = new SequentialAction(
                new SleepAction(startWaitTime),
                moveForward.build(),
                drive.actionBuilder(moveForwardEnd).turnTo(targetAngle+autoOffset).build(),
                shooterSpinUp,
               shootSequence,
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
        shooterRPM = shooterTop.getVelocity() / 28.0 * 60.0;
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

    @Override
    public void stop() {
        super.stop();
        robotPose = drive.localizer.getPose();
    }
    public double OdoAim(Pose2d robot) {
        Pose2d target = new Pose2d(0,0,0);

        if (alliance == Constants.Alliance.RED)  {
            target = new Pose2d((-targetX), targetY+4, 0);
        } else if (alliance == Constants.Alliance.BLUE)  {
            target = new Pose2d((-targetX), -(targetY+2.5), 0);
        } else if (alliance == Constants.Alliance.TEST)  {
            target = new Pose2d((-targetX), targetY+4, 0);
        }

        double deltaX = robot.position.x - target.position.x;
        double deltaY = robot.position.y - target.position.y;
        double headingToTarget = Math.atan2(deltaY, deltaX) + Math.toRadians(Constants.autoOffset);
        while (headingToTarget > Math.PI)
            headingToTarget -= Math.PI;
        while (headingToTarget < -Math.PI)
            headingToTarget += Math.PI;

        return headingToTarget;
    }

    class OdometryTurn implements Action
    {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }
}

