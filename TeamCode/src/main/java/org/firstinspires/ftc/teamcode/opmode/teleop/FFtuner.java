package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
@TeleOp
@Config
    public class FFtuner extends RobotHardware {
    public static double power = 0;

    @Override
    public void loop() {
        super.loop();


        double rpm = shooterTop.getVelocity() / 28.0 * 60.0;
        shooterTop.setPower(power);
        shooterBottom.setPower(power);

        displayData("Shooter RPM", rpm);

        // Reset gyro angle if triangle is pressed
        if (driver1.triangleOnce()) {
            imu.resetYaw();
        }


        double y = Math.pow(-driver1.left_stick_y, 3);
        double x = Math.pow(driver1.left_stick_x * 1.1, 3);
        double rx = Math.pow(driver1.right_stick_x, 3);

        // Value the motor speeds are multiplied by either 1 or the slow mode percent

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator  ;
        double rearLeftPower = (y - x + rx) / denominator ;
        double frontRightPower = (y - x - rx) / denominator  ;
        double rearRightPower = (y + x - rx) / denominator ;

        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(rearLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(rearRightPower);

        displayData("IMU angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }
}
