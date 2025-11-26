package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;

@Config
public final class RobotConstants {

    public static final PwmControl.PwmRange AXON_CONTINUOUS_PWM =
            new PwmControl.PwmRange(500, 2500, 5000);

    public static final PwmControl.PwmRange AXON_PWM =
            new PwmControl.PwmRange(520, 2480, 5000);
    public static final double MOTOR_CACHE_TOLERANCE = 0.02;
    public static final double DEADZONE = 0.2;
}