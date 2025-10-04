package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.Controller;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimer;

public class RobotHardware extends OpMode {

    protected DcMotorEx frontLeft, frontRight, rearLeft, rearRight;
    protected DcMotorEx shooterTop, shooterBottom;
    protected DcMotorEx intake;
    protected IMU imu;

    protected Controller driver1, driver2;
    protected final FtcDashboard dashboard = FtcDashboard.getInstance();
    protected TelemetryPacket packet = new TelemetryPacket();
    ElapsedTimer timer;

    /**
     * Initializes all hardware components and their configurations.
     * Sets up motor directions, modes, and zero power behaviors.
     * Configures the IMU with the orientation parameters.
     */
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        rearLeft = hardwareMap.get(DcMotorEx.class, "RearLeftDrive");
        rearRight = hardwareMap.get(DcMotorEx.class, "RearRightDrive");

        intake = hardwareMap.get(DcMotorEx.class, "Intake");

        shooterTop = hardwareMap.get(DcMotorEx.class, "ShooterTop");
        shooterBottom = hardwareMap.get(DcMotorEx.class, "ShooterBottom");

        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        imu = hardwareMap.get(IMU.class, "IMU");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        timer = new ElapsedTimer();

        driver1 = new Controller(gamepad1);
        driver2 = new Controller(gamepad2);
    }

    @Override
    public void init_loop() {
        super.init_loop();

        updateControllers();
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
        timer.updatePeriodTime();
        displayData("Loop Time", timer.getAveragePeriodSec());
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
}
