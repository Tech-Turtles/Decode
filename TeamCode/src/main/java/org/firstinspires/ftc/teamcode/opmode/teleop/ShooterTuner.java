package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.utility.Constants.kStatic;
import static org.firstinspires.ftc.teamcode.utility.Constants.kV;
import static org.firstinspires.ftc.teamcode.utility.Constants.shooterD;
import static org.firstinspires.ftc.teamcode.utility.Constants.shooterI;
import static org.firstinspires.ftc.teamcode.utility.Constants.shooterP;
import static org.firstinspires.ftc.teamcode.utility.Constants.tolerance;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.PIDController;

@TeleOp
@Config
public class ShooterTuner extends RobotHardware {
    protected PIDController shooterPID = new PIDController(shooterP, shooterI, shooterD);
    private double prevP, prevI, prevD;
    // Set point is RPM
    public static double setpoint = 0;

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

        double combined = Math.min(1, Math.max(-1, setpoint * kV + power));

        shooterTop.setPower(combined + Math.signum(power) * kStatic);
        shooterBottom.setPower(combined + Math.signum(power) * kStatic);

        displayData("Shooter RPM", rpm);
        displayData("Setpoint", setpoint);
        displayData("PID Power", power);
        displayData("kV power", setpoint * kV);

    }
}
