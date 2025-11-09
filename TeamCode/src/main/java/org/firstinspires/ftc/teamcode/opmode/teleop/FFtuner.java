package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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



    }
}
