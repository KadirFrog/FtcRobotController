package org.firstinspires.ftc.teamcode.hardware.sensor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.LastPositionStorage;
import org.firstinspires.ftc.teamcode.math.Position;

public class TrackPosition {

    public TrackPosition(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    public native void init(HardwareMap hardwareMap);

    public void stopTrackingAndStoreData() {
        stopTracking();
        Position position = getPosition();
        LastPositionStorage.storeData(new Position(position.x, position.y, getRotationRad()), getExtraDegrees());
    }

    public void writeStoredDataToVariables() {
        Position lastPosition = LastPositionStorage.getLastPosition();
        writeStoredDataToVariables(lastPosition.x, lastPosition.y, lastPosition.rotation, LastPositionStorage.getLastExtraDegrees());
    }

    public void setPosition(Position position) {
        setPosition(position.x, position.y, position.rotation);
    }

    public native void setPosition(double positionX, double positionY, double rotation);

    public native void setCoordinates(double positionX, double positionY);

    public native double getForwardMm();

    public native double getSidewardsMm();

    public native double getXPosition();

    public native double getYPosition();

    public native double getRotation();

    public native double getRotationRad();

    public native double getForwardSpeed();

    public native double getSidewardsSpeed();

    public native double getXSpeed();

    public native double getYSpeed();


    public native double getRotationSpeed();

    public Position getPosition() {
        return new Position(getXPosition(), getYPosition(), getRotation());
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("loop time", getLoopTime());
        telemetry.addData("Forward mm", getForwardMm());
        telemetry.addData("Sidewards mm", getSidewardsMm());
        telemetry.addData("X", getXPosition());
        telemetry.addData("Y", getYPosition());
        telemetry.addData("Robot Rotation Rad", getRotationRad());
        telemetry.addData("Robot rotation Degree", getRotation());
        //telemetry.addData("Speed Forward", getXSpeed());
        //telemetry.addData("Speed Sidewards", getYSpeed());
        //telemetry.addData("Robot rotation Speed", getRotationSpeed());
    }

    // private
    private native void stopTracking();

    private native int getExtraDegrees();

    private native int getLoopTime();

    private native void writeStoredDataToVariables(double positionX, double positionY, double rotation, int extraDegrees);
}
