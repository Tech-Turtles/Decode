package org.firstinspires.ftc.teamcode.utility;

import static org.firstinspires.ftc.teamcode.opmode.teleop.ShooterTuner.shooterD;
import static org.firstinspires.ftc.teamcode.opmode.teleop.ShooterTuner.shooterI;
import static org.firstinspires.ftc.teamcode.opmode.teleop.ShooterTuner.shooterP;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.List;

@Config
public final class Constants {
    public static double lowTriangle = 4775;
    public static double highTriangleEnd = 4100;
    public static double highTriangleMid = 4000;
    public static double highTriangleClose = 2200;
    public static double gateOpen = 0.68;
    public static double gateClosed = 1;
    public static double gateOpenDurationSeconds = 0.3-0.025;
    public static double redLLAngleOffset = 2.5;
    public static double blueLLAngleOffset = 7.0;
    public static double llP = 0.035, llI = 0, llD = 0.00075;
    public final double shooterP = 0.003, shooterI = 0, shooterD = 0;
    public static double superSlowMode = 0.25;
    public static double slowModeSpeed = 0.5;
    public static double llAngleSetpoint = 0;

}