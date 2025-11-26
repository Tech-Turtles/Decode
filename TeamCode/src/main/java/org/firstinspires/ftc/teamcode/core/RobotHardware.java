package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.hardware.Controller;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareStatus;
import org.firstinspires.ftc.teamcode.utility.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.utility.math.ElapsedTimer;
import org.firstinspires.ftc.teamcode.utility.roadrunner.OTOSLocalizer;

/**
 * Main robot class which is responsible for:
 *  - Initializing hardware devices defined in {@link RobotConfiguration}
 *  - Manages hub caching
 *  - Updates controller inputs
 *  - Keeps track of loop times
 *  - Sends telemetry & acme-dashboard packet data
 *  - Initializes drive train objects
 *  - Stops all applicable devices on stop()
 * </p>
 *  This class is meant to act as a base class for all OpModes run
 */
public class RobotHardware extends OpMode {

    // Voltage sensor for the control hub
    public VoltageSensor batteryVoltageSensor;
    // Timer to keep track of the period & period mean times.
    public final ElapsedTimer period = new ElapsedTimer();
    // Controller objects that act as a more intuitive wrapper for the FTC gamepad class.
    public Controller primary, secondary;
    private FtcDashboard dashboard;
    public static TelemetryPacket packet;
    protected CommandScheduler scheduler;
    protected OTOSLocalizer localizer;

    @Override
    public void init() {
        scheduler = new CommandScheduler();

        // Initialize and configure acme-dashboard objects
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        packet = new TelemetryPacket();
        // Set telemetry mode
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
        // Get control hub voltage sensor
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        // Initialize all hardware devices in RobotConfiguration & ignore errors due to null devices
        for(RobotConfiguration robotConfiguration : RobotConfiguration.values()) {
            HardwareDevice device = robotConfiguration.getAsHardwareDevice();
            try {
                device.initialize(hardwareMap.get(device.getDeviceClass(), device.getConfigName()));
            } catch (IllegalArgumentException ignore) {}
        }
        // Clear hub caches
        RobotConfiguration.CONTROL_HUB.getAsExpansionHub().clearBulkCache();

        localizer = new OTOSLocalizer(hardwareMap, new Pose2d(0,0,0));

        // Initialize controller objects with their respective gamepads
        primary = new Controller(gamepad1);
        secondary = new Controller(gamepad2);

        period.reset();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        scheduler.run();

        // Clear hub cache
        RobotConfiguration.CONTROL_HUB.getAsExpansionHub().clearBulkCache();
        // Report any devices registered as missing
        for(RobotConfiguration robotConfiguration : RobotConfiguration.values()) {
            HardwareDevice device = robotConfiguration.getAsHardwareDevice();
            if(device.getStatus().equals(HardwareStatus.MISSING)) {
                telemetry.addData(robotConfiguration.name() + " Missing; Config Name: ", device.getConfigName());
                if(packet != null)
                    packet.put(robotConfiguration.name() + " Missing; Config Name: ", device.getConfigName());
            }
        }
        // Check if the packet should be sent
        if(packet != null) {
            dashboard.sendTelemetryPacket(packet);
            packet = new TelemetryPacket();
        }
        // Update controller inputs
        primary.update();
        secondary.update();
        // Update period list
        period.updatePeriodTime();
    }

    @Override
    public void start() {
        super.start();
        // Reset packet if it is being used
        if(packet != null)
            packet = new TelemetryPacket();
        // Clear hub cache
        RobotConfiguration.CONTROL_HUB.getAsExpansionHub().clearBulkCache();
        // Reset period timer & clear previous periods so the average will maintain accuracy
        period.reset();
        period.clearPastPeriods();

        scheduler.cancelAll();
    }

    @Override
    public void loop() {
        scheduler.run();
        // Check if the packet should be sent
        if(packet != null) {
            dashboard.sendTelemetryPacket(packet);
            packet = new TelemetryPacket();
            packet.put("Period Average", period.getAveragePeriodSec());
        }
        // Output average period (loop time)
//        telemetry.addData("Period Average","%.4f sec", period.getAveragePeriodSec());
        // Clear hub cache
        RobotConfiguration.CONTROL_HUB.getAsExpansionHub().clearBulkCache();
        // Update controllers
        primary.update();
        secondary.update();
        // Update period list
        period.updatePeriodTime();
    }

    @Override
    public void stop() {
        // Check if the packet should be sent
        if(packet != null)
            dashboard.sendTelemetryPacket(packet);
        packet = null;
        dashboard = null;
        // Clear hub cache
        RobotConfiguration.CONTROL_HUB.getAsExpansionHub().clearBulkCache();
        // Run stop function for certain devices that could continue moving
        for(RobotConfiguration robotConfiguration : RobotConfiguration.values()) {
            HardwareDevice device = robotConfiguration.getAsHardwareDevice();
            if(device instanceof Motor)
                ((Motor) device).setPower(0.0);
            else if(device instanceof ContinuousServo)
                ((ContinuousServo) device).setPower(0.0);
//            else if(device instanceof Servo)
//                ((Servo) device).disablePWM();
            else if(device instanceof Webcam)
                ((Webcam) device).stop();
        }
    }

    /**
     * Helper function for setting the font color on HTML Telemetry
     * @param color Color of the text
     * @param text Text to be colored
     * @return HTML-formatted string
     */
    public static String setFontColor(String color, String text) {
        return "<font color=\""+color+"\">"+text+"</font>";
    }

    /**
     * Helper function for setting the header size on HTML Telemetry
     * @param headerNumber HTML Header value
     * @param text Text to be resized
     * @return HTML-formatted string
     */
    public static String setHeader(int headerNumber, String text) {
        return "<h"+headerNumber+">"+text+"</h"+headerNumber+">";
    }
}