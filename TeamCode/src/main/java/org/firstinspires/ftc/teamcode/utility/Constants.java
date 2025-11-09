package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class Constants {
    public static double lowTriangle = 4775;
    public static double highTriangleEnd = 4100;
    public static double highTriangleMid = 4000;
    public static double highTriangleClose = 2200;
    public static double gateOpen = 0.68;
    public static double gateClosed = 1;
    public static double gateOpenDurationSeconds = 0.3-0.025;
    public static double redLLAngleOffset = 3;
    public static double blueLLAngleOffset = 0;
    public static double llP = 3.0 , llI = 0.7, llD = 0.0;
    public static double shooterP = 0.003, shooterI = 0, shooterD = 0;
    public static double kV = 0.000169;
    public static double tolerance = 75;
    public static double kStatic = 0.06;
    public static double superSlowMode = 0.25;
    public static double slowModeSpeed = 0.5;
    public static double llAngleSetpoint = 0;
    public static double robotHalfWidth = (16.0 +(7/8.0))/2.0;
    public static double robotHalfLength = (16.5)/2.0;

}