package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RackAndPinion {
    double power = .5;
    private final DcMotor RPrightMotor;
    private final DcMotor RPleftMotor;

    public RackAndPinion(HardwareMap hardwareMap) {
        RPrightMotor = hardwareMap.get(DcMotor.class, "rp_right");
        RPrightMotor.setTargetPosition(0);
        RPrightMotor.setPower(0);
        RPrightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RPleftMotor = hardwareMap.get(DcMotor.class, "rp_left");
        RPleftMotor.setTargetPosition(0);
        RPleftMotor.setPower(0);
        RPleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RPleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void extend() {
        RPrightMotor.setTargetPosition(3000);
        RPleftMotor.setTargetPosition(3000);
    }

    public void retract() {
        RPrightMotor.setTargetPosition(0);
        RPleftMotor.setTargetPosition(0);
    }

    public void run() {
        RPrightMotor.setPower(power);
        RPleftMotor.setPower(power);
    }

    public void align() {
        int position = RPrightMotor.getCurrentPosition();
        RPrightMotor.setTargetPosition(position);
        RPleftMotor.setTargetPosition(position);
    }

    public void setPower(double power) {
        RPrightMotor.setPower(power);
        RPleftMotor.setPower(power);
    }
}
