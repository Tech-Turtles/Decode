package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utility.Constants.targetX;
import static org.firstinspires.ftc.teamcode.utility.Constants.targetY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.Controller;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimer;

public class RobotHardware extends OpMode {

    // Shooter motors
    protected DcMotorEx shooterTop, shooterBottom;
    // Intake motor
    protected DcMotorEx intake;
    protected ServoImplEx gate;
    protected Limelight3A limelight;
    protected MecanumDrive drive;
    protected Controller driver1, driver2;
    protected final FtcDashboard dashboard = FtcDashboard.getInstance();
    protected TelemetryPacket packet = new TelemetryPacket();
    ElapsedTimer timer;

    protected ServoImplEx kickerLeft, kickerRight;

    /**
     * Initializes all hardware components and their configurations.
     * Sets up motor directions, modes, and zero power behaviors.
     * Configures the IMU with the orientation parameters.
     */
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "Intake");

        gate = hardwareMap.get(ServoImplEx.class,"Gate");

        shooterTop = hardwareMap.get(DcMotorEx.class, "ShooterTop");
        shooterBottom = hardwareMap.get(DcMotorEx.class, "ShooterBottom");

        kickerLeft = hardwareMap.get(ServoImplEx.class, "KickerLeft");
        kickerRight = hardwareMap.get(ServoImplEx.class, "KickerRight");

        shooterTop.setDirection(DcMotorSimple.Direction.REVERSE);
        kickerRight.setDirection(Servo.Direction.REVERSE);

        shooterTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        timer = new ElapsedTimer();

        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
    }

    @Override
    public void init_loop() {
        super.init_loop();

        updateControllers();
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
        timer.updatePeriodTime();
        displayData("Loop Time", timer.getAveragePeriodSec());
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();
        displayData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        displayData("x", pose.position.x);
        displayData("y", pose.position.y);
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
    }

    @Override
    public void start() {
        super.start();
        timer.clearPastPeriods();
    }

    @Override
    public void loop() {

        updateControllers();
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
        timer.updatePeriodTime();
        displayData("Loop Time", timer.getAveragePeriodSec());
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();
        displayData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        displayData("x", pose.position.x);
        displayData("y", pose.position.y);
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);

    }

    @Override
    public void stop() {
        super.stop();
    }

    /**
     * Update both controllers with the latest inputs from the gamepad object.
     * This should be called in some periodic function like init_loop() or loop().
     */
    private void updateControllers()
    {
        driver1.update();
        driver2.update();
    }

    /**
     * Displays data on both the Driver Station telemetry and the FTC Dashboard.
     *
     * @param caption Caption or label for the data.
     * @param data    Data value to display.
     */
    public void displayData(String caption, Object data) {
        telemetry.addData(caption, data);
        packet.put(caption, data);
    }

    public static void drawTarget(Canvas c, Pose2d target, Pose2d robot) {
        final double TARGET_RADIUS = 2;

        c.setStrokeWidth(1);
        c.strokeCircle(target.position.x, target.position.y, TARGET_RADIUS);

        Vector2d p1 = robot.position;
        Vector2d p2 = target.position;
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
}
