package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareDevice;
import org.firstinspires.ftc.teamcode.hardware.meta.HardwareStatus;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Rotation3d;

//ToDo Add IMU axes order and signs as a constant in RobotConstants
//ToDo Add configure & set methods for axes mapping
public class HubIMU extends HardwareDevice {

    private IMU imu;
    private Quaternion cachedQuat = null;

    private Rotation3d yawOffset = new Rotation3d();

    private final int initTimeoutMs;
    private final RevHubOrientationOnRobot orientation;

    public HubIMU(String configName, RevHubOrientationOnRobot orientation, int timeout) {
        super(configName, IMU.class);
        this.orientation = orientation;
        this.initTimeoutMs = timeout;
    }

    @Override
    public void initialize(Object rawDevice) {
        if (!(rawDevice instanceof IMU)) {
            setStatus(HardwareStatus.MISSING);
            return;
        }

        this.imu = (IMU) rawDevice;

        IMU.Parameters params = new IMU.Parameters(orientation);

        imu.initialize(params);

        waitForValidQuaternion();
        setStatus(HardwareStatus.SUCCESS);
    }

    private void waitForValidQuaternion() {
        ElapsedTime timer = new ElapsedTime();
        Quaternion q;

        do {
            q = imu.getRobotOrientationAsQuaternion();
            if (q.acquisitionTime != 0) {
                this.cachedQuat = q;
                return;
            }
        } while (timer.milliseconds() < initTimeoutMs);

        //ToDo: Handle if IMU fails
    }

    public void update() {
        if (getStatus() == HardwareStatus.MISSING) return;

        Quaternion q = imu.getRobotOrientationAsQuaternion();
        if (q.acquisitionTime != 0) {
            cachedQuat = q;
        }
    }

    public Rotation3d getRotation3d() {
        if (cachedQuat == null) {
            return new Rotation3d();
        }

        return new Rotation3d(
                new org.firstinspires.ftc.teamcode.utility.math.geometry.Quaternion(
                        cachedQuat.w,
                        cachedQuat.x,
                        cachedQuat.y,
                        cachedQuat.z
                )
        ).minus(yawOffset);
    }

    public double getHeading() {
        return normalizeAngle(getRotation3d().getZ());
    }

    public void resetYaw() {
        yawOffset = getRotation3d();
    }

    public static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    public double getXAngularVelocity() {
        if (getStatus() == HardwareStatus.MISSING) return 0.0;

        AngularVelocity vel = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        return vel != null ? vel.xRotationRate : 0.0;
    }

    public double getYAngularVelocity() {
        if (getStatus() == HardwareStatus.MISSING) return 0.0;

        AngularVelocity vel = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        return vel != null ? vel.yRotationRate : 0.0;
    }

    public double getZAngularVelocity() {
        if (getStatus() == HardwareStatus.MISSING) return 0.0;

        AngularVelocity vel = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        return vel != null ? vel.zRotationRate : 0.0;
    }
}