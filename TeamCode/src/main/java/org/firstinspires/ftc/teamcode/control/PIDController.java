package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.math.Vec3d;

@SuppressWarnings("unused")
public class PIDController {

    /*
     * PID Controller
     * Based on https://en.wikipedia.org/wiki/PID_controller
     */

    // Proportional gain
    double kp;
    // Integral gain
    double ki;
    // Derivative gain
    double kd;

    double integral = 0;
    double previousError = 0;

    long lastTimeMillis;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        lastTimeMillis = System.currentTimeMillis();
    }

    public double calculatePIDAlgorithm(double error) {
        double deltaT = (System.currentTimeMillis() - lastTimeMillis) / 1000.0;
        integral += error * deltaT;

        double output = kp * error + ki * integral + kd * ((error - previousError) / deltaT);
        previousError = error;
        lastTimeMillis = System.currentTimeMillis();
        return output;
    }

    public void resetIntegral() {
        lastTimeMillis = System.currentTimeMillis();
        integral = 0;
    }

    public Vec3d getPIDGains() {
        return new Vec3d(kp, ki, kd);
    }

    public void setPIDGains(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double getKp() {
        return kp;
    }

    public void setKp(double kp) {
        this.kp = kp;
    }

    public double getKi() {
        return ki;
    }

    public void setKi(double ki) {
        this.ki = ki;
    }

    public double getKd() {
        return kd;
    }

    public void setKd(double kd) {
        this.kd = kd;
    }
}