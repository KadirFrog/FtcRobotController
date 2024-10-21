package org.firstinspires.ftc.teamcode.hardware.sensor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control.LastPositionStorage;
import org.firstinspires.ftc.teamcode.math.Position;

@SuppressWarnings("unused")
public class DeadWheelOdometry {
    /* FAKE NEWS!!!!
     * We use 1 encoder for forward / backward
     * and 2 encoders for sidewards / turning (sidewards is averaged from these two to not change when the robot turns)
     */
    DcMotorEx motorForwardEncoder;
    DcMotorEx motorSidewardsEncoder;
    int motorForwardEncoderValue = 0;
    int motorSidewardsEncoderValue = 0;
    int encoderForwardLastPos = 0;
    int encoderSidewardsLastPos = 0;

    final static double DISTANCE_ENCODER_SIDEWARDS_MID = 0;
    final static double DISTANCE_ENCODER_FORWARD_MID = 0;

    final static double ENCODER_TICKS_PER_ROTATION = 8192;
    final static double WHEEL_DIAMETER_MM = 60;
    final static double ENCODER_TICKS_CORRECTED_SIDEWARDS = 2 * Math.PI * (DISTANCE_ENCODER_SIDEWARDS_MID * (ENCODER_TICKS_PER_ROTATION / (WHEEL_DIAMETER_MM * Math.PI)));
    final static double ENCODER_TICKS_CORRECTED_FORWARD = 2 * Math.PI * (DISTANCE_ENCODER_FORWARD_MID * (ENCODER_TICKS_PER_ROTATION / (WHEEL_DIAMETER_MM * Math.PI)));

    double robotPositionY = 0;
    double robotPositionX = 0;
    double robotRotationRad = 0;
    double storedRad = 0;
    double robotRotationDeg = 0;
    double robotRotationDegContinuous = 0;
    int extraDegrees = 0;

    long lastTimeMillis = System.currentTimeMillis();
    double forwardSpeed = 0;
    double sidewardsSpeed = 0;
    double forwardSpeedIMU = 0;
    double sidewardsSpeedIMU = 0;
    double ySpeed = 0;
    double xSpeed = 0;
    double rotationSpeed = 0;

    int loopTime = 0;

    public DeadWheelOdometry(HardwareMap hardwareMap) {
        motorForwardEncoder = hardwareMap.get(DcMotorEx.class, "intake");
        motorSidewardsEncoder = hardwareMap.get(DcMotorEx.class, "dead_wheel_sidewards");

        motorSidewardsEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorForwardEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorSidewardsEncoder.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public Runnable backgroundTask = () -> {
        while (Robot.state != Robot.State.SHOULD_STOP) {
            Robot.controlHub.getBulkData();
            Robot.expansionHub.getBulkData();
            if (Robot.controlHub.isNotResponding()) {
                Robot.errorDetection.addErrorToErrorLog("control hub did not respond");
            }
            if (Robot.expansionHub.isNotResponding()) {
                Robot.errorDetection.addErrorToErrorLog("expansion hub did not respond");
            }

            double previousRobotRotationDeg = robotRotationDeg;
            double previousRobotPositionY = robotPositionY;
            double previousRobotPositionX = robotPositionX;

            double rotationRadNotFinal = -Robot.imu1.getRotation().z + storedRad;
            if (rotationRadNotFinal > Math.PI) {
                robotRotationRad = rotationRadNotFinal - Math.PI * 2;
            } else if (rotationRadNotFinal < -Math.PI) {
                robotRotationRad = rotationRadNotFinal + Math.PI * 2;
            } else {
                robotRotationRad = rotationRadNotFinal;
            }

            motorForwardEncoderValue = motorForwardEncoder.getCurrentPosition(); // Division by zero - (int) ((robotRotationDegContinuous / 360) * ENCODER_TICKS_CORRECTED_FORWARD);
            motorSidewardsEncoderValue = motorSidewardsEncoder.getCurrentPosition(); // Division by zero - (int) ((robotRotationDegContinuous / 360) * ENCODER_TICKS_CORRECTED_SIDEWARDS);

            robotRotationDeg = Math.toDegrees(robotRotationRad);
            // convert degrees to continuous degrees system

            if(previousRobotRotationDeg > 90 && robotRotationDeg < -90) {
                extraDegrees += 360;
            } else if (previousRobotRotationDeg < -90 && robotRotationDeg > 90) {
                extraDegrees -= 360;
            }
            robotRotationDegContinuous = robotRotationDeg + extraDegrees;

            double deltaT = (System.currentTimeMillis() - lastTimeMillis) / 1000.0;

            double sin = Math.sin(robotRotationRad);
            double cos = Math.cos(robotRotationRad);

            double deltaForward = encoderForwardLastPos - motorForwardEncoderValue;
            double deltaSidewards = encoderSidewardsLastPos - motorSidewardsEncoderValue;

            double deltaY = ((-deltaForward * cos + deltaSidewards * sin) / ENCODER_TICKS_PER_ROTATION) * WHEEL_DIAMETER_MM * Math.PI;
            double deltaX = ((-deltaForward * sin - deltaSidewards * cos) / ENCODER_TICKS_PER_ROTATION) * WHEEL_DIAMETER_MM * Math.PI;

            robotPositionY += deltaY;
            robotPositionX += deltaX;

            forwardSpeed = ((deltaForward / deltaT) / ENCODER_TICKS_PER_ROTATION * WHEEL_DIAMETER_MM * Math.PI + forwardSpeed) / 2;
            sidewardsSpeed = ((deltaSidewards / deltaT) / ENCODER_TICKS_PER_ROTATION * WHEEL_DIAMETER_MM * Math.PI + sidewardsSpeed) / 2;
            ySpeed = (((robotPositionY - previousRobotPositionY) / deltaT) / ENCODER_TICKS_PER_ROTATION * WHEEL_DIAMETER_MM * Math.PI + ySpeed) / 2;
            xSpeed = (((robotPositionX - previousRobotPositionX) / deltaT) / ENCODER_TICKS_PER_ROTATION * WHEEL_DIAMETER_MM * Math.PI + xSpeed) / 2;
            rotationSpeed = (((robotRotationDeg - previousRobotRotationDeg) / deltaT) + rotationSpeed) / 2;

            encoderForwardLastPos = motorForwardEncoderValue;
            encoderSidewardsLastPos = motorSidewardsEncoderValue;

            //forwardSpeedIMU = Robot.imu1.getAcceleration().y * deltaT * 1000;
            //sidewardsSpeedIMU = Robot.imu1.getAcceleration().x * deltaT * 1000;

           /*if ((Robot.errorDetection.detectFire("dead_wheel_sidewards", motorSidewardsEncoder.getVelocity(), sidewardsSpeedIMU) || Robot.errorDetection.detectFire("dead_wheel_forwards", motorForwardEncoder.getVelocity(), forwardSpeedIMU)) && firstTime) {
               firstTime = false;
               Robot.driveToPositionController.deactivate();
           }*/

            loopTime = (int) (System.currentTimeMillis() - lastTimeMillis);
            lastTimeMillis = System.currentTimeMillis();
        }
    };

    public void storeData() {
        LastPositionStorage.storeData(new Position(robotPositionX, robotPositionY, robotRotationRad), extraDegrees);
    }

    public void writeStoredDataToVariables() {
        Position lastPosition = LastPositionStorage.getLastPosition();
        robotPositionX = lastPosition.x;
        robotPositionY = lastPosition.y;
        storedRad = lastPosition.rotation;
        extraDegrees = LastPositionStorage.getLastExtraDegrees();
    }

    public void setPosition(Position position) {
        robotPositionX = position.x;
        robotPositionY = position.y;
        robotRotationDegContinuous = position.rotation;
    }

    public void setPosition(double robotPositionX, double robotPositionY, double robotRotation) {
        this.robotPositionX = robotPositionX;
        this.robotPositionY = robotPositionY;
        this.robotRotationDegContinuous = robotRotation;
    }

    public double getYPosition() {
        return robotPositionY;
    }

    public double getXPosition() {
        return robotPositionX;
    }

    public double getRotation() {
        return robotRotationDegContinuous;
    }

    public double getRotationRad() {
        return robotRotationRad;
    }

    public double getXSpeed() {
        return xSpeed;
    }

    public double getYSpeed() {
        return ySpeed;
    }

    public double getRotationSpeed() {
        return rotationSpeed;
    }

    public double getForwardMm() {
        return motorForwardEncoderValue / ENCODER_TICKS_PER_ROTATION * WHEEL_DIAMETER_MM * Math.PI;
    }

    public double getSidewardsMm() {
        return motorSidewardsEncoderValue / ENCODER_TICKS_PER_ROTATION * WHEEL_DIAMETER_MM * Math.PI;
    }

    public double getForwardSpeed() {
        return forwardSpeed;
    }

    public double getSidewardsSpeed() {
        return sidewardsSpeed;
    }

    public Position getPosition() {
        return new Position(robotPositionX, robotPositionY, robotRotationDegContinuous);
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("loop time" , loopTime);
        telemetry.addData("Forward mm", getForwardMm());
        telemetry.addData("Sidewards mm", getSidewardsMm());
        telemetry.addData("Robot Rotation Rad", robotRotationRad);
        telemetry.addData("Robot rotation Degree", robotRotationDegContinuous);
        telemetry.addData("Robot rotation Speed", rotationSpeed);
        telemetry.addData("X", robotPositionX);
        telemetry.addData("Y", robotPositionY);
        telemetry.addData("Speed Forward", xSpeed);
        telemetry.addData("Speed Sidewards", ySpeed);
        telemetry.addData("extraDegrees", extraDegrees);
    }
}