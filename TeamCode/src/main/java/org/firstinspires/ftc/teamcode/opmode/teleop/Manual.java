package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimer;
import org.firstinspires.ftc.teamcode.utility.PIDController;

@TeleOp
@Config
public class Manual extends RobotHardware {
    public static double slowModeSpeed = 0.5;
    private boolean slowOn = false;
    private final double shooterP = 0.0045, shooterI = 0, shooterD = 0;
    protected PIDController shooterPID = new PIDController(shooterP, shooterI, shooterD);
    // Set point is RPM
    private final double tolerance = 20;
    public static double kStatic = 0.06;
    private double setpoint = 0;
    public static double lowTriangle = 4900;
    public static double highTriangleEnd = 3500;
    public static double highTriangleMid = 2500;
    public static double highTriangleClose = 2200;

    public static double gateOpen = 0.68;

    public static double gateClosed = 1;

    public static double gateOpenDurationSeconds = 0.3;

    private ElapsedTimer gateTimer = new ElapsedTimer();
    private boolean gateTimerActive;

    @Override
    public void init() {
        super.init();

    }

    @Override
    public void init_loop() {
        super.init_loop();

    }

    @Override
    public void start() {
        super.start();
        shooterPID.setPID(shooterP, shooterI, shooterD);
        shooterPID.setTolerance(tolerance);
    }

    @Override
    public void loop() {
        super.loop();

        // Toggle slow mode on or off if right bumper is pressed
        if (driver1.rightBumperOnce()) {
            slowOn = !slowOn;
        }

        // If slow mode is on then update the slow speed multiplier
        double slowSpeed = 1;
        if (slowOn) {
           slowSpeed = slowModeSpeed;
        }

        double y = Math.pow(-driver1.left_stick_y, 3);
        double x = Math.pow(driver1.left_stick_x * 1.1, 3);
        double rx = Math.pow(driver1.right_stick_x, 3);

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) * slowSpeed / denominator;
        double rearLeftPower = (rotY - rotX + rx) * slowSpeed / denominator;
        double frontRightPower = (rotY - rotX - rx) * slowSpeed / denominator;
        double rearRightPower = (rotY + rotX - rx) *slowSpeed / denominator;

        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(rearLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(rearRightPower);

        // Intake control - Forward and backwards
        double intakeF = driver1.left_trigger;
        double intakeB = driver1.right_trigger;
        intake.setPower(intakeF - intakeB);

        // Driver 2 shooter controls
        if (driver2.triangle()) {
            setpoint = lowTriangle;
        } else if (driver2.square()) {
            setpoint = highTriangleEnd;
        } else if (driver2.cross()) {
            setpoint = highTriangleMid;
        } else if (driver2.circle()) {
            setpoint = highTriangleClose;
        } else {
            setpoint = 0;
        }

        if (driver2.rightBumper()){
            gate.setPosition(gateOpen);
        } else if (driver2.leftBumperOnce()) {
            gateTimerActive = true;
            gateTimer.reset();
            gate.setPosition(gateOpen);
        } else if (gateTimerActive) {
            if (gateTimer.seconds() > gateOpenDurationSeconds)
            {
                gateTimerActive = false;
                gate.setPosition(gateClosed);
            }
        } else {
            gate.setPosition(gateClosed);
        }

        // Calculate shooter rpm; ticks per second to rpm
        // 6000 rpm motor is 28 ticks per rotation
        double shooterRPM = shooterTop.getVelocity() / 28.0 * 60.0;
        double power = shooterPID.calculate(shooterRPM, setpoint);
        shooterTop.setPower(power + Math.signum(power) * kStatic);
        shooterBottom.setPower(power + Math.signum(power) * kStatic);

        // Output shooter calculations to driver station & dashboard
        displayData("Shooter RPM",shooterRPM);
        displayData("Setpoint RPM", setpoint);
        displayData("PID Power", power);
        displayData("Gate Timer (sec)", gateTimer.seconds());
        displayData("Heading (deg)", Math.toDegrees(botHeading));
    }
}
