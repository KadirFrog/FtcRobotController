/*package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MatchData;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.sensor.camera.TeamPropDetection;
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.vision.VisionPortal;

@SuppressWarnings({"unused", "ConstantConditions"})
@Autonomous(name = "Auto adjustable at init", group = "!Important")
public class AutonomousAdjustableAtInit extends Robot {
        boolean startCloseToBackdrop = MatchData.startCloseToBackdrop;
        boolean redAlliance = MatchData.redAlliance;
        boolean scorePixelAtLeftBackdropPosition = MatchData.scorePixelAtLeftBackdropPosition;

        @Override
        public void runOpMode() throws InterruptedException {
            init(hardwareMap);

            Position startingPosition;
            double targetXTeamProp = 500;
            if (startCloseToBackdrop && redAlliance) {
                startingPosition = new Position(1800.0, 0.0, 0.0);
                targetXTeamProp = 1700;
            } else if (!startCloseToBackdrop && redAlliance) {
                startingPosition = new Position(600.0, 0.0, 0.0);
            } else if (startCloseToBackdrop && !redAlliance) {
                startingPosition = new Position(-1650.0, 0.0, 0.0);
                targetXTeamProp = 1700;
            } else {
                startingPosition = new Position(-450.0, 0.0, 0.0);
            }
            trackPosition.setPosition(startingPosition);
            TeamPropDetection teamPropDetection = new TeamPropDetection();

            visionPortal = new VisionPortal.Builder().setCamera(front_webcam).addProcessors(drawFrame, teamPropDetection).setCameraResolution(new Size(640, 480)).setAutoStopLiveView(true).build();

            driveToPositionController.setMaxDeviation(20, 20, 5);
            driveToPositionController.activate();
            addExecutor(() -> {
                while (!isStopRequested()) {
                    trackPosition.printTelemetry(telemetry);
                    telemetry.update();
                }
                state = State.SHOULD_STOP;
            });
            pixelsInRobot = 2;

            inputManager.gamepad1.a.addButtonPressListener(() -> redAlliance = !redAlliance);
            inputManager.gamepad1.b.addButtonPressListener(() -> startCloseToBackdrop = !startCloseToBackdrop);
            inputManager.gamepad1.x.addButtonPressListener(() -> scorePixelAtLeftBackdropPosition = !scorePixelAtLeftBackdropPosition);

            state = State.INITIALIZED;
            telemetry.addLine("Initialized");
            telemetry.update();

            while (opModeInInit()) {
                inputManager.update(gamepad1, gamepad2);
                telemetry.addData("a: red alliance", redAlliance);
                telemetry.addData("b: start close to backdrop", startCloseToBackdrop);
                telemetry.addData("x: score pixel at left backdrop position", scorePixelAtLeftBackdropPosition);
                telemetry.update();
            }

            waitForStart();
            if (opModeIsActive()) {
                state = State.AUTONOMOUS;

                visionPortal.setProcessorEnabled(drawFrame, false);
                int positionTeamProp = teamPropDetection.getPositionTeamProp();
                visionPortal.setProcessorEnabled(teamPropDetection, false);

                if (!redAlliance) {
                    if (positionTeamProp == 0) {
                        positionTeamProp = 2;
                    } else if (positionTeamProp == 2) {
                        positionTeamProp = 0;
                    }
                }

                double targetYTeamProp = 750;
                if (startCloseToBackdrop) {
                    if (positionTeamProp == 0) {
                        driveToPositionController.setTargetPosition(targetXTeamProp, targetYTeamProp, 0.0);
                        driveToPositionController.waitUntilPositionIsReached(2000);
                        driveToPositionController.setTargetPosition(targetXTeamProp, targetYTeamProp, 90.0);
                    } else if (positionTeamProp == 1) {
                        driveToPositionController.setTargetPosition(targetXTeamProp, targetYTeamProp, 0.0);
                        driveToPositionController.waitUntilPositionIsReached(2000);
                        driveToPositionController.setTargetPosition(targetXTeamProp, targetYTeamProp, 180.0);
                    } else {
                        driveToPositionController.setTargetPosition(targetXTeamProp + 150, 1500.0, 0.0);
                        driveToPositionController.waitUntilPositionIsReached(2000);
                        driveToPositionController.setTargetPosition(targetXTeamProp, targetYTeamProp, 0.0);
                        driveToPositionController.waitUntilPositionIsReached(2000);
                        driveToPositionController.setTargetPosition(targetXTeamProp, targetYTeamProp, -90.0);
                    }
                } else {
                    if (positionTeamProp == 0) {
                        driveToPositionController.setTargetPosition(targetXTeamProp, targetYTeamProp, 0.0);
                        driveToPositionController.waitUntilPositionIsReached(2000);
                        driveToPositionController.setTargetPosition(targetXTeamProp, targetYTeamProp, 90.0);
                    } else if (positionTeamProp == 1) {
                        driveToPositionController.setTargetPosition(targetXTeamProp, 1100.0, 0.0);
                    } else {
                        driveToPositionController.setTargetPosition(targetXTeamProp, targetYTeamProp, 0.0);
                        driveToPositionController.waitUntilPositionIsReached(2000);
                        driveToPositionController.setTargetPosition(targetXTeamProp, targetYTeamProp, -90.0);
                    }
                }

                driveToPositionController.waitUntilPositionIsReached(2000);
                driveToPositionController.deactivate();
                //intake.deactivateScanningPixel();
                rackAndPinion.setPower(0.3);
                Thread.sleep(2000);
                rackAndPinion.deactivateIntake();
                pixelsInRobot--;
                driveToPositionController.activate();
                //intake.activateScanningPixel();
                if (startCloseToBackdrop && positionTeamProp != 0) {
                    driveToPositionController.setTargetPosition(targetXTeamProp, targetYTeamProp, 90.0);
                    driveToPositionController.waitUntilPositionIsReached(2000);
                } else if (!startCloseToBackdrop && positionTeamProp != 1) {
                    driveToPositionController.setTargetPosition(targetXTeamProp, targetYTeamProp, 0.0);
                    driveToPositionController.waitUntilPositionIsReached(2000);
                }

                double targetY = 600;
                if ((positionTeamProp == 0 && redAlliance) || (positionTeamProp == 2 && !redAlliance)) {
                    targetY = 448;
                } else if ((positionTeamProp == 2 && redAlliance) || (positionTeamProp == 0 && !redAlliance)) {
                    targetY = 752;
                }

                if ((scorePixelAtLeftBackdropPosition && redAlliance) || (!scorePixelAtLeftBackdropPosition && !redAlliance)) {
                    targetY += 76;
                }

                //Thread.sleep(millisecondsToWaitBeforeDrivingToBackdrop);
                if (!startCloseToBackdrop) {
                    driveToPositionController.setTargetPosition(600.0, 1300.0, 0.0);
                    driveToPositionController.waitUntilPositionIsReached(2000);
                    driveToPositionController.setTargetPosition(600.0, 1300.0, 90.0);
                    driveToPositionController.waitUntilPositionIsReached(2000);
                    driveToPositionController.setTargetPosition(2200.0, 1300.0, 90.0);
                    driveToPositionController.waitUntilPositionIsReached(2000);
                    driveToPositionController.setTargetPosition(2200.0, 1300.0, 0.0);
                    driveToPositionController.waitUntilPositionIsReached(1000);
                    driveToPositionController.setTargetPosition(2200.0, targetY, 0.0);
                    driveToPositionController.waitUntilPositionIsReached(2000);
                }
                driveToPositionController.setTargetPosition(2200.0, targetY, 90.0);
                driveToPositionController.waitUntilPositionIsReached(1000);
                driveToPositionController.setTargetPosition(2650.0, targetY, 90.0);
                driveToPositionController.waitUntilPositionIsReached(2000);
                driveToPositionController.deactivate();

            /*elevator.setTargetElevationToLayer(0);
            outputAndArm.makeOutputReadyForOutput();
            while (!outputAndArm.finishedRotatingArm() && !isStopRequested());
            while (!elevator.isAtTargetPosition() && !isStopRequested());
            outputAndArm.releasePixel();

            Thread.sleep(1000);

            elevator.setTargetElevationInmm(0);
            outputAndArm.makeOutputReadyForIntake();
            while (!outputAndArm.finishedRotatingArm() && !isStopRequested());
    /*        while (!elevator.isAtTargetPosition() && !isStopRequested());*/
       /*         if (startCloseToBackdrop) {
                    driveToPositionController.activate();
                    driveToPositionController.setTargetPosition(2500.0, 0.0, 90.0);
                    driveToPositionController.waitUntilPositionIsReached(8000);
                    driveToPositionController.deactivate();
                }
            }

            stopRobot();
        }
}*/
