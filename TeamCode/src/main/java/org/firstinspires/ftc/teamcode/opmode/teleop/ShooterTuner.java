package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.PIDController;

@TeleOp
@Config
public class ShooterTuner extends RobotHardware {

    public static double shooterP = 0.0045, shooterI = 0, shooterD = 0;
    private boolean slowModeEnabled = false;
    private static final double DRIVE_SLOW_MODE_MULTIPLIER = 0.25;
    protected PIDController shooterPID = new PIDController(shooterP, shooterI, shooterD);
    private double prevP, prevI, prevD;
    // Set point is RPM
    public static double tolerance = 20, setpoint = 0;
    public static double kStatic = 0.06;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();

        if(prevP != shooterP || prevI != shooterI || prevD != shooterD || shooterPID.getPositionTolerance() != tolerance)
        {
            shooterPID.setPID(shooterP, shooterI, shooterD);
            shooterPID.setTolerance(tolerance);

            prevP = shooterP;
            prevI = shooterI;
            prevD = shooterD;
        }
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        if(prevP != shooterP || prevI != shooterI || prevD != shooterD || shooterPID.getPositionTolerance() != tolerance)
        {
            shooterPID.setPID(shooterP, shooterI, shooterD);
            shooterPID.setTolerance(tolerance);

            prevP = shooterP;
            prevI = shooterI;
            prevD = shooterD;
        }

        double rpm = shooterTop.getVelocity() / 28.0 * 60.0;
        double power = shooterPID.calculate(rpm, setpoint);

        shooterTop.setPower(power + Math.signum(power) * kStatic);
        shooterBottom.setPower(power + Math.signum(power) * kStatic);

        displayData("Shooter RPM", rpm);
        displayData("Setpoint", setpoint);
        displayData("PID Power", power);

        // Reset gyro angle if triangle is pressed
        if (driver1.triangleOnce()) {
            imu.resetYaw();
        }

        // Toggle slow mode on or off if cross is pressed
        if (driver1.crossOnce()) {
            slowModeEnabled = !slowModeEnabled;
        }

        double y = Math.pow(-driver1.left_stick_y, 3);
        double x = Math.pow(driver1.left_stick_x * 1.1, 3);
        double rx = Math.pow(driver1.right_stick_x, 3);

        // Value the motor speeds are multiplied by either 1 or the slow mode percent
        double slowMode = 1.0;
        if (slowModeEnabled)
            slowMode = DRIVE_SLOW_MODE_MULTIPLIER;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator * slowMode;
        double rearLeftPower = (y - x + rx) / denominator * slowMode;
        double frontRightPower = (y - x - rx) / denominator * slowMode;
        double rearRightPower = (y + x - rx) / denominator * slowMode;

        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(rearLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(rearRightPower);

        displayData("IMU angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }
}
