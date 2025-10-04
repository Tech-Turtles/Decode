package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
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
    public static double lowTriangle = 4300;
    public static double highTriangleEnd = 3500;
    public static double highTriangleMid = 2500;
    public static double highTriangleClose = 2200;

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

        // Slow mode toggle on/off
        if (driver1.rightBumperOnce()) {
            slowOn = !slowOn;
        }

        // If slow mode is on then update the slow speed multiplier
        double slowSpeed = 1;
        if (slowOn) {
           slowSpeed = slowModeSpeed;
        }

        double straight = -driver1.left_stick_y * slowSpeed;
        double rotation = driver1.right_stick_x * slowSpeed;
        double strafe   = driver1.left_stick_x * slowSpeed;
        double maxSpeed = Math.max(Math.abs(straight) + Math.abs(rotation) + Math.abs(strafe), 1.0);
        frontLeft.setPower( (straight + rotation + strafe) / maxSpeed);
        frontRight.setPower((straight - rotation - strafe) / maxSpeed);
        rearLeft.setPower(  (straight + rotation + strafe) / maxSpeed);
        rearRight.setPower( (straight - rotation - strafe) / maxSpeed);

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

        // Calculate shooter rpm; ticks per second to rpm
        // 6000 rpm motor is 28 ticks per rotation
        double shooterRPM = shooterTop.getVelocity() * 28.0 / 60.0;
        double power = shooterPID.calculate(shooterRPM, setpoint);
        shooterTop.setPower(power + Math.signum(power) * kStatic);
        shooterBottom.setPower(power + Math.signum(power) * kStatic);

        // Output shooter calculations to driver station & dashboard
        displayData("Shooter RPM",shooterRPM);
        displayData("Setpoint", setpoint);
        displayData("PID Power", power);
    }
}
