package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.utility.Constants.blueLLAngleOffset;
import static org.firstinspires.ftc.teamcode.utility.Constants.flipperAdd;
import static org.firstinspires.ftc.teamcode.utility.Constants.gateClosed;
import static org.firstinspires.ftc.teamcode.utility.Constants.gateOpen;
import static org.firstinspires.ftc.teamcode.utility.Constants.gateOpenDurationSeconds;
import static org.firstinspires.ftc.teamcode.utility.Constants.highTriangleClose;
import static org.firstinspires.ftc.teamcode.utility.Constants.highTriangleEnd;
import static org.firstinspires.ftc.teamcode.utility.Constants.highTriangleMid;
import static org.firstinspires.ftc.teamcode.utility.Constants.kStatic;
import static org.firstinspires.ftc.teamcode.utility.Constants.kV;
import static org.firstinspires.ftc.teamcode.utility.Constants.kicked;
import static org.firstinspires.ftc.teamcode.utility.Constants.kicking;
import static org.firstinspires.ftc.teamcode.utility.Constants.llAngleSetpoint;
import static org.firstinspires.ftc.teamcode.utility.Constants.llD;
import static org.firstinspires.ftc.teamcode.utility.Constants.llI;
import static org.firstinspires.ftc.teamcode.utility.Constants.llP;
import static org.firstinspires.ftc.teamcode.utility.Constants.lowTriangle;
import static org.firstinspires.ftc.teamcode.utility.Constants.maxRotationalVel;
import static org.firstinspires.ftc.teamcode.utility.Constants.redLLAngleOffset;
import static org.firstinspires.ftc.teamcode.utility.Constants.robotHalfWidth;
import static org.firstinspires.ftc.teamcode.utility.Constants.robotHalfLength;
import static org.firstinspires.ftc.teamcode.utility.Constants.rotationKs;
import static org.firstinspires.ftc.teamcode.utility.Constants.rotationKv;
import static org.firstinspires.ftc.teamcode.utility.Constants.shooterD;
import static org.firstinspires.ftc.teamcode.utility.Constants.shooterI;
import static org.firstinspires.ftc.teamcode.utility.Constants.shooterP;
import static org.firstinspires.ftc.teamcode.utility.Constants.slowModeSpeed;
import static org.firstinspires.ftc.teamcode.utility.Constants.superSlowMode;
import static org.firstinspires.ftc.teamcode.utility.Constants.targetX;
import static org.firstinspires.ftc.teamcode.utility.Constants.targetY;
import static org.firstinspires.ftc.teamcode.utility.Constants.tolerance;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimer;
import org.firstinspires.ftc.teamcode.utility.PIDController;

import java.util.ArrayList;
import java.util.List;
@TeleOp
@Config
public class Manual extends RobotHardware {

    protected PIDController shooterPID = new PIDController(shooterP, shooterI, shooterD);
    // Set point is RPM
    private double setpoint = 0;
    private final ElapsedTimer gateTimer = new ElapsedTimer();
    private boolean gateTimerActive;
    List<Integer> shotRPMList = new ArrayList<>();

    private double prevP, prevI, prevD;
    protected PIDController llAnglePID = new PIDController(llP, llI, llD);
    private static int allianceColor = 0; //0 being blue 1 being red
    private double fieldDriveOffsetDeg = -90;
    public static boolean robotCentric = false;

    @Override
    public void init() {
        super.init();
        llAnglePID.enableContinuousInput(-Math.PI, Math.PI);
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

        double x = Math.pow(-driver1.left_stick_y, 3);
        double y = Math.pow(driver1.left_stick_x * 1.1, 3);
        double rx = Math.pow(driver1.right_stick_x, 3);

        if (driver1.dpadUpOnce() && allianceColor <= 0) {
            allianceColor = 1;
            driver1.setLedColor(255,1, 0, -1);
            llAngleSetpoint = redLLAngleOffset;
            fieldDriveOffsetDeg = -90;
        } else if (driver1.dpadUpOnce() && allianceColor >= 1) {
            allianceColor = 0;
            driver1.setLedColor(0, 20, 255, -1);
            llAngleSetpoint = blueLLAngleOffset;
            fieldDriveOffsetDeg = 90;
        }


        if (prevP != llP || prevI != llI || prevD != llD )
        {
            llAnglePID.setPID(llP, llI, llD);

            prevP = llP;
            prevI = llI;
            prevD = llD;
        }

        if (driver1.crossOnce())
              drive.localizer.setPose(new Pose2d((72- robotHalfWidth), (-72 + robotHalfLength), Math.toRadians(90)));

        Pose2d robot = drive.localizer.getPose();
        Pose2d target;
        double offset;

        if (allianceColor == 1)
        {
            target = new Pose2d(-(targetX), targetY, 0);
            offset = redLLAngleOffset;
        }
        else
        {
            target = new Pose2d(-(targetX), -(targetY), 0);
            offset = blueLLAngleOffset;
        }
        
        packet.fieldOverlay().setStroke("#4FF1B5");
        drawTarget(packet.fieldOverlay(), target, robot);
        double deltaX = robot.position.x - target.position.x;
        double deltaY = robot.position.y - target.position.y;
        double headingToTarget = Math.atan2(deltaY, deltaX) + Math.toRadians(offset);
        while (headingToTarget > Math.PI)
            headingToTarget -= Math.PI;
        while (headingToTarget < -Math.PI)
            headingToTarget += Math.PI;
        telemetry.addData("Target Pose", "X: %.2f Y: %.2f", target.position.x, target.position.y);
        telemetry.addData("Delta Pose", "X: %.2f Y: %.2f", deltaX, deltaY);
        displayData("Heading To Target", Math.toDegrees(headingToTarget));

        if (driver1.right_trigger >= 0.5)
        {
            double error = headingToTarget-robot.heading.toDouble();
            double ff = 0;
            double kV = rotationKv*(error * maxRotationalVel);
            if (Math.abs(error) > 0.001)
                ff = rotationKs * Math.signum(error) + kV;
            rx = (llAnglePID.calculate(headingToTarget, robot.heading.toDouble()) - ff);
        }
        displayData("rx", rx);

//        LLResult result = limelight.getLatestResult();
//        if (result.isValid()) {
//            double captureLatency = result.getCaptureLatency();
//            double targetingLatency = result.getTargetingLatency();
//            double parseLatency = result.getParseLatency();
//            telemetry.addData("LL Latency", captureLatency + targetingLatency);
//            telemetry.addData("Parse Latency", parseLatency);
//            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
//
//            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//            for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
//
//                if (fr.getFiducialId() == 24 || fr.getFiducialId() == 20)
//                {
//                    if (driver1.right_trigger >= 0.5) {
//                        rx = -(llAnglePID.calculate(fr.getTargetXDegrees(), llAngleSetpoint));
//                        displayData("Limelight rotation", rx);
//                    }
//
//                }
//            }
//        }

        if (gamepad1.ps) {
            drive.localizer.setPose(new Pose2d(0,0,0));
            fieldDriveOffsetDeg = 0;
        }
        
        if (driver1.triangleOnce())
            robotCentric = !robotCentric;

        if (robotCentric)
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(x*slowSpeed, -y*slowSpeed), rx*slowSpeed));
        else
            drive.setDrivePowersField(x*slowSpeed, -y*slowSpeed, rx*slowSpeed, fieldDriveOffsetDeg);


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
                gate.setPosition(gateClosed);
                if (gateTimer.seconds() > gateOpenDurationSeconds + flipperAdd) {
                    gateTimerActive = false;
                    kickerLeft.setPosition(kicking);
                    kickerRight.setPosition(kicking);
                } else {
                    kickerLeft.setPosition(kicked);
                    kickerRight.setPosition(kicked);
                }
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
        displayData("Shooter RPM on everyshot", shotRPMList);
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
    }
}
