/*
package org.firstinspires.ftc.teamcode.opmodes.remotecontrolled;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MatchData;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.control.LastPositionStorage;
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.vision.VisionPortal;

@SuppressWarnings({"unused", "BusyWait"})
@TeleOp(name = "Die wahre Sünde", group = "!Important")
public class RemoteControlledFlorian extends Robot {
    double rotationOffsetRad = 0;
    boolean driveFast = false;
    int layer = 0;

    int buttonsPressedForSwitchingHeadlessOnOff = 0;

    boolean driveToPositionMode = false;
    boolean driveToBackdrop = false;

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        driveToPositionController.setMaxDeviation(100, 100, 20);
        driveToPositionController.setMaxMotorPower(1);

        if (LastPositionStorage.validDataAvailable()) {
            trackPosition.writeStoredDataToVariables();
        }

        visionPortal = new VisionPortal.Builder().setCamera(front_webcam).setCameraResolution(new Size(640, 480)).setAutoStopLiveView(true).build();


        //GAMEPAD 1


        inputManager.gamepad1.both_sticks.addUpdateListener((left_x, left_y, right_x, right_y) -> {
            if (!driveToPositionMode && !driveToBackdrop) {
                    if (driveFast) {
                        drivetrain.drive(left_y, left_x, right_x, 1, 1);
                    } else {
                        drivetrain.drive(left_y, left_x, right_x, 0.4, 0.4);
                    }
            }
        });
        inputManager.gamepad1.left_stick.addUpdateListener((x, y) -> {
            if (!driveToPositionMode && driveToBackdrop) {
                if (Math.abs(x) > 0.1) {
                    drivetrain.driveWithoutAnyEnhancements(0, x, 0).holdDesiredDistanceForwards(100);
                } else {
                    drivetrain.driveToDesiredDistance(100);
                }
            }
        });
        inputManager.gamepad1.left_stick_button.addButtonPressListener(() -> {
            rotationOffsetRad = trackPosition.getRotationRad();
            drivetrain.setRotationOffset(rotationOffsetRad);
        });


        inputManager.gamepad1.right_trigger.addFullPressListener(value -> raiseButtonsPressedForSwitchingHeadlessOnOff());
        inputManager.gamepad1.right_trigger.addTriggerReleaseListener(value -> buttonsPressedForSwitchingHeadlessOnOff--);

        inputManager.gamepad1.left_trigger.addFullPressListener(value -> raiseButtonsPressedForSwitchingHeadlessOnOff());
        inputManager.gamepad1.left_trigger.addTriggerReleaseListener(value -> buttonsPressedForSwitchingHeadlessOnOff--);

        inputManager.gamepad1.right_bumper.addButtonPressListener(this::raiseButtonsPressedForSwitchingHeadlessOnOff);
        inputManager.gamepad1.right_bumper.addButtonReleaseListener(() -> buttonsPressedForSwitchingHeadlessOnOff--);

        inputManager.gamepad1.left_bumper.addButtonPressListener(this::raiseButtonsPressedForSwitchingHeadlessOnOff);
        inputManager.gamepad1.left_bumper.addButtonReleaseListener(() -> buttonsPressedForSwitchingHeadlessOnOff--);


        inputManager.gamepad1.right_trigger.addFullPressListener(value -> driveFast = true);
        inputManager.gamepad1.right_trigger.addTriggerReleaseListener(value -> driveFast = false);

        inputManager.gamepad1.left_trigger.addFullPressListener(value -> {
            drivetrain.stop();
            driveToPositionMode = true;
        });
        inputManager.gamepad1.left_trigger.addTriggerReleaseListener(value -> {
            drivetrain.stop();
            driveToPositionMode = false;
            driveToPositionController.deactivate();
            rackAndPinion.deactivateIntake();
        });


        inputManager.gamepad1.right_bumper.addButtonPressListener(() -> rackAndPinion.activateIntake());
        inputManager.gamepad1.right_bumper.addButtonReleaseListener(() -> rackAndPinion.deactivateIntake());

        inputManager.gamepad1.left_bumper.addButtonPressListener(() -> rackAndPinion.activateIntakeReversed());
        inputManager.gamepad1.left_bumper.addButtonReleaseListener(() -> rackAndPinion.deactivateIntake());


        inputManager.gamepad1.a.addButtonPressListener(() -> {
            if (driveToPositionMode) {
            } else {
                driveToBackdrop = true;
            }
        });
        inputManager.gamepad1.a.addButtonReleaseListener(() -> {
            driveToBackdrop = false;
            driveToPositionController.deactivate();
        });

        inputManager.gamepad1.b.addButtonPressListener(() -> {
            if (driveToPositionMode) {
                trackPosition.setCoordinates(600 * (MatchData.redAlliance ? 1 : -1), 1300);
            } else {
                drivetrain.useDriveMotorEncoderSupport(!drivetrain.getUseDriveMotorEncoderSupport());
            }
        });
        inputManager.gamepad1.b.addButtonReleaseListener(() -> driveToPositionController.deactivate());



        inputManager.gamepad1.x.addButtonPressListener(() -> {
            if (driveToPositionMode) {
                addExecutor(() -> {
                    double rotationDegCon = trackPosition.getRotation();
                    double rotationRad = trackPosition.getRotationRad();
                    double diffDegConDeg = rotationDegCon - rotationDegCon % 360;

                    final Position target1 = new Position(600.0, 1300.0, diffDegConDeg - 90);
                    final Position target2 = new Position(2400.0, target1.y, target1.rotation);
                    final Position target3 = new Position(target2.x, target2.y, diffDegConDeg - 45);
                    final Position target4 = new Position(2800.0, 900.0, target3.rotation);
                    final Position target5 = new Position(target2.x, target2.y, diffDegConDeg + 90);
                    final Position target6 = new Position(target1.x, target1.y, diffDegConDeg + 90);

                    int state = 0;
                    driveToPositionController.activate();
                    while (driveToPositionMode && state != 7 && !isStopRequested()) {
                        if (state == 0) {
                            driveToPositionController.setTargetPosition(target1);
                            state = 1;
                        }
                        if (driveToPositionController.isPositionReached()) {
                            switch (state) {
                                case 1:
                                    driveToPositionController.setTargetPosition(target2);
                                    state = 2;
                                    break;
                                case 2:
                                    driveToPositionController.setTargetPosition(target3);
                                    state = 3;
                                    break;
                                case 3:
                                    driveToPositionController.setTargetPosition(target4);
                                    state = 4;
                                    break;
                                case 4:
                                    rackAndPinion.activateIntakeReversed();
                                    driveToPositionController.setTargetPosition(target2);
                                    state = 5;
                                    break;
                                case 5:
                                    rackAndPinion.deactivateIntake();
                                    driveToPositionController.setTargetPosition(target5);
                                    state = 6;
                                    break;
                                case 6:
                                    driveToPositionController.setTargetPosition(target6);
                                    state = 7;
                                    break;
                            }
                        }
                        try {
                            Thread.sleep(20);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    }
                });
            } else {
                drivetrain.useDeadWheelSupport(!drivetrain.getUseDeadWheelSupport());
            }
        });
        inputManager.gamepad1.x.addButtonReleaseListener(() -> {
            driveToPositionController.deactivate();
            rackAndPinion.deactivateIntake();
        });

        inputManager.gamepad1.y.addButtonPressListener(() -> {
            if (driveToPositionMode) {
                addExecutor(() -> {
                    double rotationDegCon = trackPosition.getRotation();
                    double rotationRad = trackPosition.getRotationRad();
                    double diffDegConDeg = rotationDegCon - rotationDegCon % 360;

                    final Position target1 = new Position(600.0, 1300.0, diffDegConDeg - 90);
                    final Position target2 = new Position(2400.0, target1.y, target1.rotation);
                    final Position target3 = new Position(target2.x, target2.y, diffDegConDeg);
                    final Position target4 = new Position(2400.0, 700.0, target3.rotation);
                    final Position target5 = new Position(target2.x, target2.y, diffDegConDeg + 90);
                    final Position target6 = new Position(target1.x, target1.y, diffDegConDeg + 90);

                    int state = 0;
                    driveToPositionController.activate();
                    while (driveToPositionMode && state != 7 && !isStopRequested()) {
                        if (state == 0) {
                            driveToPositionController.setTargetPosition(target1);
                            state = 1;
                        }
                        if (driveToPositionController.isPositionReached()) {
                            switch (state) {
                                case 1:
                                    driveToPositionController.setTargetPosition(target2);
                                    state = 2;
                                    break;
                                case 2:
                                    driveToPositionController.setTargetPosition(target3);
                                    state = 3;
                                    break;
                                case 3:
                                    driveToPositionController.setTargetPosition(target4);
                                    state = 4;
                                    break;
                                case 4:
                                    rackAndPinion.activateIntakeReversed();
                                    driveToPositionController.setTargetPosition(target2);
                                    state = 5;
                                    break;
                                case 5:
                                    rackAndPinion.deactivateIntake();
                                    driveToPositionController.setTargetPosition(target5);
                                    state = 6;
                                    break;
                                case 6:
                                    driveToPositionController.setTargetPosition(target6);
                                    state = 7;
                                    break;
                            }
                        }
                        try {
                            Thread.sleep(20);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    }
                });
            } else {
                drivetrain.useGyroSupport(!drivetrain.getUseGyroSupport());
            }
        });
        inputManager.gamepad1.y.addButtonReleaseListener(() -> driveToPositionController.deactivate());

        inputManager.gamepad1.dpad_left.addButtonPressListener(() -> {
            if (suspensionArmAndDroneLauncher.getElevationInDegrees() > 10) {
                suspensionArmAndDroneLauncher.launchDrone();
            }
        });

        inputManager.gamepad1.dpad_right.addButtonPressListener(() -> suspensionArmAndDroneLauncher.setTargetElevationInDegrees(30));

        elevator.deactivate();

        drivetrain.useHeadless(true).useDeadWheelSupport(false).useGyroSupport(true).useDriveMotorEncoderSupport(false).setTimeFromZeroToMaxSpeed(500);

        pixelsInRobot = 2;
        state = State.INITIALIZED;
        telemetry.addLine("Initialized");
        telemetry.update();


        waitForStart();
        state = State.TELEOP;
        while (opModeIsActive()) {
            // execute the controller inputs
            inputManager.update(gamepad1, gamepad2);
            if (gamepad1.dpad_up) {
                suspensionArmAndDroneLauncher.setTargetElevationInDegrees(suspensionArmAndDroneLauncher.getElevationInDegrees() + 2);
            }
            if (gamepad1.dpad_down) {
                suspensionArmAndDroneLauncher.setTargetElevationInDegrees(suspensionArmAndDroneLauncher.getElevationInDegrees() - 2);
            }

            if (!driveToPositionMode) {
                drivetrain.doDrive();
            }

            // Check Encoder
            //elevator.checkEncoder();
            //suspensionArm.checkEncoder();

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
    private void raiseButtonsPressedForSwitchingHeadlessOnOff() {
        buttonsPressedForSwitchingHeadlessOnOff++;
        if (buttonsPressedForSwitchingHeadlessOnOff == 4) {
            drivetrain.useHeadless(!drivetrain.getUseHeadless());
        }
    }
}*/