package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

@SuppressWarnings("unused")
public class Elevator {

    DcMotorEx motorElevator;
    boolean encoderPluggedIn = true;

    static double ENCODER_STEPS_PER_mm = 3.1344; // calibrated; accuracy ~ +-2%
    static double MAX_ELEVATION = 800;

    double targetElevationInmm = 0;
    double targetPower = 0;

    int layer = 0;

    public Elevator(HardwareMap hardwareMap) {
        motorElevator = hardwareMap.get(DcMotorEx.class, "elevator");
        motorElevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorElevator.setDirection(DcMotorEx.Direction.REVERSE);
        motorElevator.setTargetPosition(0);
        motorElevator.setPower(0);
        motorElevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setTargetPower(double power) {
        targetPower = power;
        motorElevator.setPower(targetPower);
    }

    public void setTargetElevationInmm(double elevationInmm, double power) {
        if(elevationInmm < -100 || elevationInmm > MAX_ELEVATION || !encoderPluggedIn)
            return;
        targetElevationInmm = elevationInmm;
        targetPower = power;
        motorElevator.setTargetPosition((int) (targetElevationInmm * ENCODER_STEPS_PER_mm));
        motorElevator.setPower(targetPower);
    }

    public void setTargetElevationInmm(double elevationInmm) {
        if (!encoderPluggedIn)
            return;
        setTargetElevationInmm(elevationInmm, .5);
    }

    public void checkEncoder() {
        if (motorElevator.isBusy() && encoderPluggedIn) {
            if (Robot.errorDetection.detectFire("elevator", motorElevator.getVelocity(), targetPower, 3)) {
                encoderPluggedIn = false;
                motorElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    public double getElevationInmm() {
        return motorElevator.getCurrentPosition() / ENCODER_STEPS_PER_mm;
    }

    public double getTargetElevationInmm() {
        return targetElevationInmm;
    }

    public double getMaxElevation() {
        return MAX_ELEVATION;
    }

    public void stop() {
        motorElevator.setPower(0);
    }

    public void setTargetElevationToLayer(int layer) {
        setTargetElevationInmm(76 * layer + 10);
    }

    public void setTargetElevationToLayer() {
        setTargetElevationInmm(76 * layer + 10);
    }

    public boolean isAtTargetPosition() {
        return (getTargetElevationInmm() + 5 > getElevationInmm() && getTargetElevationInmm() - 5 < getElevationInmm());
    }

    public int getLayer() {
        return layer;
    }

    public void setLayer(int layer) {
        if (layer <= 14 && layer >= 0) {
            this.layer = layer;
        }
    }

    public void deactivate() {
        motorElevator.setMotorDisable();
    }
}
