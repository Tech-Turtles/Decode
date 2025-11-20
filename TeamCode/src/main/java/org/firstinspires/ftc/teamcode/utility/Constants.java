package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class Constants {
    public enum Alliance{
        RED, BLUE, TEST
    }
    public static double lowTriangle = 4300
            ;
    public static double highTriangleEnd = 3750;
    public static double highTriangleMid = 3750;
    public static double highTriangleClose = 3618;
    public static double gateOpen = 0.68;
    public static double gateClosed = 1;
    public static double gateOpenDurationSeconds = 0.3-0.025;
    public static double redLLAngleOffset = -0.5;
    public static double blueLLAngleOffset = 0;
    public static double llP = 0.8 , llI = 0.001, llD = 0.1;
    public static double shooterP = 0.003, shooterI = 0, shooterD = 0;
    public static double kV = 0.000169;
    public static double tolerance = 75;
    public static double kStatic = 0.06;
    public static double superSlowMode = 0.25;
    public static double slowModeSpeed = 0.5;
    public static double llAngleSetpoint = 0;
    public static double robotHalfWidth = (17.5)/2.0;
    public static double robotHalfLength = (18)/2.0;
    public static double kicking = 1;
    public static double kicked = 0;
    public static double flipperAdd = 0.4;
    public static double rotationKs = 0.09;
    public static double rotationKv = 0.08;
    public static double maxRotationalVel = 2*Math.PI;
    public static double targetX = 70;
    public static double targetY = 70;
    public static double autoOffset= 0;

}