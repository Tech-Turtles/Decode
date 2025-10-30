package org.firstinspires.ftc.teamcode.opmode.autonomous;


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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimer;
import org.firstinspires.ftc.teamcode.utility.PIDController;

import static org.firstinspires.ftc.teamcode.utility.Constants.*;

@Autonomous
@Config
public class SmallTriangleAuto extends RobotHardware {

    private MecanumDrive drive;
    private Canvas canvas;
    private Action autonomous;
    private double shooterSetPoint = 0.0;
    private boolean shouldRun;

    private static double previousTime = 0;

    private static double robotHalfW = (16.0 +(7/8.0))/2.0;

    private final double kV = 0.000169;
    private final double shooterP = 0.003, shooterI = 0, shooterD = 0;
    protected PIDController shooterPID = new PIDController(shooterP, shooterI, shooterD);
    // Set point is RPM
    private final double tolerance = 20;
    public static double kStatic = 0.06;

    public static double gateOpen = 0.68;

    public static double gateClosed = 1;

    public static double gateOpenDurationSeconds = 0.3;

    private final ElapsedTimer gateTimer = new ElapsedTimer();

    @Override
    public void init() {
        super.init();
        gate.setPosition(gateClosed);
        drive = new MecanumDrive(hardwareMap, new Pose2d(-72+robotHalfW, (0+robotHalfW), Math.toRadians(90-67.0)));

        TrajectoryActionBuilder moveForward =
                drive.actionBuilder(new Pose2d(-72+robotHalfW, (0+robotHalfW), Math.toRadians(90-67))).lineToX(-72+robotHalfW+  20);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        autonomous = new SequentialAction(
                new InstantAction(() -> //does on start, Shoot 3 balls
                {
                    shooterSetPoint = lowTriangle;
                }),
                new SleepAction(1.5+0.5),



                new InstantAction(() ->
                {
                    gate.setPosition(gateOpen);
                }),
                new SleepAction(gateOpenDurationSeconds),
                new InstantAction(() ->
                {
                    gate.setPosition(gateClosed);
                }),
                new SleepAction(1),



                new InstantAction(() ->
                {
                    gate.setPosition(gateOpen);
                }),
                new SleepAction(gateOpenDurationSeconds),
                new InstantAction(() ->
                {
                    gate.setPosition(gateClosed);
                }),
                new SleepAction(1),



                new InstantAction(() ->
                {
                    gate.setPosition(gateOpen);
                }),
                new SleepAction(gateOpenDurationSeconds),
                new InstantAction(() ->
                {
                    gate.setPosition(gateClosed);
                }),
                new SleepAction(1),



                new InstantAction(() ->
                {
                    shooterSetPoint = 0;
                }),


                moveForward.build()
        );

        canvas = new Canvas();
        autonomous.preview(canvas);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        displayData("Front Left Drive Motor Position", frontLeft.getCurrentPosition());
        displayData("Front Right Drive Motor Position", frontRight.getCurrentPosition());
        displayData("Rear Left Drive Motor Position", rearLeft.getCurrentPosition());
        displayData("Rear Right Drive Motor Position", rearRight.getCurrentPosition());
        displayData("IMU angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
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

        displayData("Shooter RPM",shooterRPM);
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

}
