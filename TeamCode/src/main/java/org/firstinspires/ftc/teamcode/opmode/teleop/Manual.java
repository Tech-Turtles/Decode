package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.opmode.teleop.ShooterTuner.shooterD;
import static org.firstinspires.ftc.teamcode.opmode.teleop.ShooterTuner.shooterI;
import static org.firstinspires.ftc.teamcode.opmode.teleop.ShooterTuner.shooterP;
import static org.firstinspires.ftc.teamcode.utility.Constants.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimer;
import org.firstinspires.ftc.teamcode.utility.PIDController;
import java.util.ArrayList;
import java.util.List;
@TeleOp
@Config
public class Manual extends RobotHardware {
    private final double kV = 0.000169;
    protected PIDController shooterPID = new PIDController(shooterP, shooterI, shooterD);
    // Set point is RPM
    private final double tolerance = 75;
    public static double kStatic = 0.06;
    private double setpoint = 0;
    private final ElapsedTimer gateTimer = new ElapsedTimer();
    private boolean gateTimerActive;
    List<Integer> shotRPMList = new ArrayList<>();

    private double prevP, prevI, prevD;
    protected PIDController llAnglePID = new PIDController(llP, llI, llD);
    private static int allianceColor = 0; //0 being blue 1 being red

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
        limelight.start();
    }

    @Override
    public void loop() {
        super.loop();

        LLStatus status = limelight.getStatus();

        // Calculate shooter rpm; ticks per second to rpm
        // 6000 rpm motor is 28 ticks per rotation
        double shooterRPM = shooterTop.getVelocity() / 28.0 * 60.0;
        double power = shooterPID.calculate(shooterRPM, setpoint);
        double combined = Math.min(1, Math.max(-1, setpoint * kV + power));

        if (shooterPID.atSetpoint() && setpoint != 0)
        {
            driver2.setLedColor(0, 255, 0, -1);
        }
        else
        {
            driver2.setLedColor(255, 0, 0, -1);
        }

        // If slow mode is on then update the slow speed multiplier
        double slowSpeed = 1;
        if (driver1.leftBumper()) {
           slowSpeed = slowModeSpeed;
        } else if (driver1.rightBumper()) {
           slowSpeed = superSlowMode;
        }

        double y = Math.pow(-driver1.left_stick_y, 3);
        double x = Math.pow(driver1.left_stick_x * 1.1, 3);
        double rx = Math.pow(driver1.right_stick_x, 3);

        if (driver1.dpadUpOnce() && allianceColor <= 0) {
            allianceColor = 1;
            driver1.setLedColor(255,5, 0, -1);
            llAngleSetpoint = redLLAngleOffset;
        } else if (driver1.dpadUpOnce() && allianceColor >= 1) {
            allianceColor = 0;
            driver1.setLedColor(0, 20, 255, -1);
            llAngleSetpoint = blueLLAngleOffset;
        }


        if (prevP != llP || prevI != llI || prevD != llD )
        {
            llAnglePID.setPID(llP, llI, llD);

            prevP = llP;
            prevI = llI;
            prevD = llD;
        }

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());

                if (fr.getFiducialId() == 24 || fr.getFiducialId() == 20)
                {
                    if (driver1.right_trigger >= 0.5) {
                        rx = -(llAnglePID.calculate(fr.getTargetXDegrees(), llAngleSetpoint));
                        displayData("Limelight rotation", rx);
                    }

                }
            }
        }

        if (gamepad1.options) {
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP));
            imu.initialize(parameters);
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
            shotRPMList.add( (int) shooterRPM);
        } else if (gateTimerActive) {
            if (gateTimer.seconds() > gateOpenDurationSeconds)
            {
                gateTimerActive = false;
                gate.setPosition(gateClosed);
            }
        } else {
            gate.setPosition(gateClosed);
        }

        if (setpoint != 0) {
            shooterTop.setPower(combined + Math.signum(power) * kStatic);
            shooterBottom.setPower(combined + Math.signum(power) * kStatic);
        } else {
            shooterTop.setPower(0);
            shooterBottom.setPower(0);
        }

        // Output shooter calculations to driver station & dashboard
        displayData("Shooter RPM",shooterRPM);
        displayData("Setpoint RPM", setpoint);
        displayData("PID Power", power);
        displayData("Gate Timer (sec)", gateTimer.seconds());
        displayData("Heading (deg)", Math.toDegrees(botHeading));
        displayData("Shooter RPM on everyshot", shotRPMList);
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
    }
}
