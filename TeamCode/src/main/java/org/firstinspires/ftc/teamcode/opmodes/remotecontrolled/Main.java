package org.firstinspires.ftc.teamcode.opmodes.remotecontrolled;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control.LastPositionStorage;
import org.firstinspires.ftc.teamcode.hardware.RackAndPinion;

@TeleOp(name = "Die 8. Todsünde", group = "!Important")
public class Main extends Robot {
    boolean driveToPositionMode = false;
    boolean driveFast = false;
    double rotationOffsetRad = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        driveToPositionController.setMaxDeviation(100, 100, 20);
        driveToPositionController.setMaxMotorPower(1);

        if (LastPositionStorage.validDataAvailable()) {
            trackPosition.writeStoredDataToVariables();
        }
        inputManager.gamepad1.right_trigger.addPressListener((value) -> {
            driveFast = false;
        });
        inputManager.gamepad1.right_trigger.addFullPressListener((value) -> {
            driveFast = true;
        });
        inputManager.gamepad1.both_sticks.addUpdateListener((left_x, left_y, right_x, right_y) -> {
                if (driveFast) {
                    drivetrain.drive(left_y, left_x, right_x, .7, 1);
                } else {
                    drivetrain.drive(left_y, left_x, right_x, 0.4, 0.6);
                }
        });
        inputManager.gamepad1.left_stick_button.addButtonPressListener(() -> {
            rotationOffsetRad = trackPosition.getRotationRad();
            drivetrain.setRotationOffset(rotationOffsetRad);
        });
        int elevator_var = 1000;
        inputManager.gamepad1.dpad_up.addButtonPressListener(() -> rackAndPinion.extend());
        inputManager.gamepad1.dpad_down.addButtonPressListener(() -> rackAndPinion.retract());
        inputManager.gamepad1.y.addButtonPressListener(() -> elevator.setTargetElevationInmm(elevator_var));
        inputManager.gamepad1.a.addButtonPressListener(() -> elevator.setTargetElevationInmm(0));
        telemetry.addLine("Init");
        telemetry.update();
        drivetrain.useHeadless(true).useDeadWheelSupport(true).useGyroSupport(false).useDriveMotorEncoderSupport(false).setTimeFromZeroToMaxSpeed(500);

        waitForStart();
        state = State.TELEOP;
        while (opModeIsActive()) {
            rackAndPinion.run();
            inputManager.update(gamepad1, gamepad2);
            drivetrain.doDrive();

            // print telemetry
            drivetrain.printStuff(telemetry);
            telemetry.addData("current target", driveToPositionController.getTargetPosition());
            telemetry.addData("isPositionReached", driveToPositionController.isPositionReached());
            errorDetection.printMotorStatus(telemetry);
            trackPosition.printTelemetry(telemetry);
            telemetry.update();

        }
        stopRobot();
    }

}
