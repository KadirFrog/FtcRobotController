package org.firstinspires.ftc.teamcode.hardware.sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.math.Vec3d;

@SuppressWarnings("unused")
public class IMU {

    BNO055IMU internalIMU;

    Vec3d.Vec3dRemapOrder axisRemapOrder;

    static BNO055IMU.Parameters defaultParameters() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        return parameters;
    }

    public IMU(BNO055IMU internalIMU, BNO055IMU.Parameters parameters, Vec3d.Vec3dRemapOrder axisRemapOrder) {
        this.internalIMU = internalIMU;
        this.internalIMU.initialize(parameters);
        this.axisRemapOrder = axisRemapOrder;
        this.internalIMU.startAccelerationIntegration(new Position(), new Velocity(), 100);
    }

    public IMU(BNO055IMU internalIMU, Vec3d.Vec3dRemapOrder axisRemapOrder) {
        this(internalIMU, defaultParameters(), axisRemapOrder);
    }

    public IMU(BNO055IMU internalIMU) {
        this(internalIMU, defaultParameters(), Vec3d.Vec3dRemapOrder.XYZ_XYZ);
    }

    public synchronized double getZRotation() {
        return internalIMU.getAngularOrientation().firstAngle
                ;
    }

    public synchronized Vec3d getRotation() {
        return new Vec3d(internalIMU.getAngularOrientation().firstAngle, internalIMU.getAngularOrientation().secondAngle, internalIMU.getAngularOrientation().thirdAngle).remap(axisRemapOrder);
    }

    public synchronized Vec3d getAcceleration() {
        return new Vec3d(internalIMU.getAcceleration().xAccel, internalIMU.getAcceleration().yAccel, internalIMU.getAcceleration().zAccel).remap(axisRemapOrder);
    }

    public synchronized Vec3d getAngularVelocity() {
        return new Vec3d(internalIMU.getAngularVelocity().xRotationRate, internalIMU.getAngularVelocity().yRotationRate, internalIMU.getAngularVelocity().zRotationRate).remap(axisRemapOrder);
    }
}
