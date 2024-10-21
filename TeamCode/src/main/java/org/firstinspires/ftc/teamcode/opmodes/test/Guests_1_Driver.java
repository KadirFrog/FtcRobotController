/*package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control.LastPositionStorage;

@SuppressWarnings("unused")
@TeleOp(name = "Vector Stiftung 1 driver", group = "Test")
public class Guests_1_Driver extends Robot {

    boolean gripperClosed = false;
    boolean armInFrontOfRobot = true;

    double rotationOffsetRad = 0;

    int coneStackHeight = 5;

    double startVolt = 0;
    double minVolt = 0;
    int count = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        startVolt = controlHub.getInputVoltage(VoltageUnit.VOLTS);

        if(LastPositionStorage.validDataAvailable()) {
            rotationOffsetRad = LastPositionStorage.getLastKnownRotationRad();
        }
        state = State.INITIALIZED;

        waitForStart();
        state = State.TELEOP;
        while (opModeIsActive()) {
            if (startVolt - 0.5 < controlHub.getInputVoltage(VoltageUnit.VOLTS)) {
                minVolt = ((minVolt * count + controlHub.getInputVoltage(VoltageUnit.VOLTS)) / (count + 1));
                count++;
            }

            if (controlHub.getInputVoltage(VoltageUnit.VOLTS) < 6) {
                stop();
                stop();
            }

            double rotationRadCorrected = trackPosition.getRobotRotationRad() + rotationOffsetRad;
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0.4 + 0.03 * (13 - minVolt), 0.7 + 0.03 * (13 - minVolt));

            if (gamepad1.left_stick_button) {
                rotationOffsetRad -= rotationRadCorrected;
            }

            drivetrain.doDrive();

            trackPosition.printTelemetry(telemetry);

            inputManager.update(gamepad1, gamepad2);
        }
        stopRobot();
    }

}*/

