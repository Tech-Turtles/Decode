package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.AbsoluteEncoder;
import org.firstinspires.ftc.teamcode.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.hardware.Encoder;
import org.firstinspires.ftc.teamcode.hardware.ExpansionHub;
import org.firstinspires.ftc.teamcode.hardware.HubIMU;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MotorTypes;
import org.firstinspires.ftc.teamcode.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;

public enum RobotConfiguration {
    IMU(
            new HubIMU("IMU 1",
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    ),
                    500
            )
    ),
    CONTROL_HUB(
            new ExpansionHub("Expansion Hub 173")
            .configureBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
    ),
    EXPANSION_HUB(
            new ExpansionHub("Expansion Hub 2")
            .configureBulkCachingMode(LynxModule.BulkCachingMode.OFF)
    ),
    DRIVE_FRONT_LEFT(
            new Motor("Front Left Drive")
            .configureDirection(DcMotorSimple.Direction.REVERSE)
            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .setType(MotorTypes.DRIVE)
    ),
    DRIVE_FRONT_RIGHT(
            new Motor("Front Right Drive")
            .configureDirection(DcMotorSimple.Direction.FORWARD)
            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .setType(MotorTypes.DRIVE)
    ),
    DRIVE_BACK_LEFT(
            new Motor("Back Left Drive")
            .configureDirection(DcMotorSimple.Direction.REVERSE)
            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .setType(MotorTypes.DRIVE)
    ),
    DRIVE_BACK_RIGHT(
            new Motor("Back Right Drive")
            .configureDirection(DcMotorSimple.Direction.FORWARD)
            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .setType(MotorTypes.DRIVE)
    ),
    ODOMETRY_PARALLEL(
            new Encoder("Front Right Drive")
            .setDirection(Encoder.Direction.REVERSE)
    ),
    ODOMETRY_PERPENDICULAR(
            new Encoder("Back Left Drive")
            .setDirection(Encoder.Direction.REVERSE)
    ),
    INTAKE_1(
            new Motor("Intake 1")
            .configureDirection(DcMotorSimple.Direction.FORWARD)
            .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
            .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            .setType(MotorTypes.OTHER)
    ),
    INTAKE_2(
            new Motor("Intake 2")
                    .configureDirection(DcMotorSimple.Direction.REVERSE)
                    .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                    .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    .setType(MotorTypes.OTHER)
    ),
    SHOOTER_TOP(
            new Motor("Shooter Top")
                    .configureDirection(DcMotorSimple.Direction.FORWARD)
                    .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                    .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    .setType(MotorTypes.OTHER)
    ),
    SHOOTER_BOTTOM(
            new Motor("Shooter Bottom")
                    .configureDirection(DcMotorSimple.Direction.REVERSE)
                    .configureZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
                    .configureRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    .setType(MotorTypes.OTHER)
    ),
    HOOD(
            new Servo("Hood")
                    .configureDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD)
    ),
    TURRET(
            new ContinuousServo("Turret")
                    .configureDirection(DcMotorSimple.Direction.FORWARD)
                    .configurePIDWrapping()
    ),
    TRANSFER_1(
            new ContinuousServo("Transfer 1")
                    .configureDirection(DcMotorSimple.Direction.FORWARD)
    ),
    TRANSFER_2(
            new ContinuousServo("Transfer 2")
                    .configureDirection(DcMotorSimple.Direction.FORWARD)
    );

    private final HardwareDevice device;

    RobotConfiguration(HardwareDevice device) {
        this.device = device;
    }

    public HardwareDevice getAsHardwareDevice() {
        return device;
    }

    public HubIMU getAsIMU() {
        if(!(device instanceof HubIMU))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (HubIMU) device;
    }

    public Motor getAsMotor() {
        if(!(device instanceof Motor))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (Motor) device;
    }

    public Servo getAsServo() {
        if(!(device instanceof Servo))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (Servo) device;
    }

    public ContinuousServo getAsContinuousServo() {
        if(!(device instanceof ContinuousServo))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (ContinuousServo) device;
    }

    public AbsoluteEncoder getAsAbsoluteEncoder() {
        if(!(device instanceof AbsoluteEncoder))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (AbsoluteEncoder) device;
    }

    public ExpansionHub getAsExpansionHub() {
        if(!(device instanceof ExpansionHub))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (ExpansionHub) device;
    }

    public Encoder getAsEncoder() {
        if(!(device instanceof Encoder))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (Encoder) device;
    }
    
    public Webcam getAsWebcam() {
        if(!(device instanceof Webcam))
            throw new IllegalArgumentException("Attempt to retrieve hardware component as incorrect type.");
        return (Webcam) device;
    }
}