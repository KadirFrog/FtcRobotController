/*package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Size;

import org.firstinspires.ftc.teamcode.MatchData;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.sensor.camera.TeamPropDetection;
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.vision.VisionPortal;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Match Nr. " + MatchData.matchNumber, group = "!Important")
@SuppressWarnings("ConstantConditions")
public class Autonomous extends Robot {
    final boolean startCloseToBackdrop = MatchData.startCloseToBackdrop;
    final boolean redAlliance = MatchData.redAlliance;
    final boolean scorePixelAtLeftBackdropPosition = MatchData.scorePixelAtLeftBackdropPosition;
    final boolean alliancePartnerIsScoringAtBackdrop = MatchData.alliancePartnerIsScoringAtBackdrop;
    final boolean alliancePartnerIsScoringOnSpikeMark = MatchData.alliancePartnerIsScoringOnSpikeMark;
    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        Position startingPosition;
        double targetXTeamPropFarFromBackdrop = 500;
        double targetXTeamPropCloseToBackdrop = 1800;
        if (startCloseToBackdrop && redAlliance) {
            startingPosition = new Position(1800.0, 0.0, 0.0); //Close to backdrop & red alliance
        } else if (!startCloseToBackdrop && redAlliance) {
            startingPosition = new Position(600.0, 0.0, 0.0); //Far from Backdrop & red alliance
        } else if (startCloseToBackdrop && !redAlliance) {
            startingPosition = new Position(-1650.0, 0.0, 0.0); //Close to backdrop & blue alliance
        } else {
            startingPosition = new Position(-450.0, 0.0, 0.0); //Far from Backdrop & blue alliance
        }
        trackPosition.setPosition(startingPosition);
        TeamPropDetection teamPropDetection = new TeamPropDetection();

        visionPortal = new VisionPortal.Builder().setCamera(front_webcam).addProcessors(drawFrame, teamPropDetection).setCameraResolution(new Size(640, 480)).setAutoStopLiveView(true).build();

        driveToPositionController.setMaxDeviation(20, 20, 5);
        driveToPositionController.activate();

        drivetrain.useHeadless(false).useGyroSupport(false).useDeadWheelSupport(false).useDriveMotorEncoderSupport(false).useControlledAcceleration(true);

        addExecutor(() -> {
            while (!isStopRequested());
            state = State.SHOULD_STOP;
        });
        pixelsInRobot = 2;
        state = State.INITIALIZED;
        telemetry.addLine("Initialized");
        telemetry.update();
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
            if (startCloseToBackdrop) { //Place Pixel at SpikeMark
                if (positionTeamProp == 0) {
                    driveToPositionController.setTargetPosition(targetXTeamPropCloseToBackdrop, targetYTeamProp, 0.0).waitUntilPositionIsReached(4000);
                    driveToPositionController.setTargetPosition(targetXTeamPropCloseToBackdrop, targetYTeamProp, 90.0);
                } else if (positionTeamProp == 1) {
                    driveToPositionController.setTargetPosition(targetXTeamPropCloseToBackdrop, targetYTeamProp - 100, 0.0).waitUntilPositionIsReached(4000);
                    driveToPositionController.setTargetPosition(targetXTeamPropCloseToBackdrop, targetYTeamProp - 100, 180.0);
                } else {
                    double targetXThisOne = targetXTeamPropCloseToBackdrop + 500;
                    driveToPositionController.setTargetPosition(targetXTeamPropCloseToBackdrop, 300.0, 0.0).waitUntilPositionIsReached(4000);
                    driveToPositionController.setTargetPosition(targetXThisOne, 300.0, 0.0).waitUntilPositionIsReached(4000);
                    driveToPositionController.setTargetPosition(targetXThisOne, targetYTeamProp, 0.0).waitUntilPositionIsReached(4000);
                    driveToPositionController.setTargetPosition(targetXThisOne, targetYTeamProp, 90.0).waitUntilPositionIsReached(4000);
                }
            } else {
                if (positionTeamProp == 0) {
                    driveToPositionController.setTargetPosition(targetXTeamPropFarFromBackdrop - 300, targetYTeamProp + 300, 0.0).waitUntilPositionIsReached(2000);
                } else if (positionTeamProp == 1) {
                    driveToPositionController.setTargetPosition(targetXTeamPropFarFromBackdrop, targetYTeamProp + 500, 0.0);
                } else {
                    driveToPositionController.setTargetPosition(targetXTeamPropFarFromBackdrop, targetYTeamProp, 0.0).waitUntilPositionIsReached(2000);
                    driveToPositionController.setTargetPosition(targetXTeamPropFarFromBackdrop, targetYTeamProp, -90.0);
                }
            }

            driveToPositionController.waitUntilPositionIsReached(2000);
            driveToPositionController.deactivate();
            rackAndPinion.setPower(-0.3);
            Thread.sleep(500);
            rackAndPinion.deactivateIntake();
            pixelsInRobot--;
            driveToPositionController.activate();

            if(startCloseToBackdrop && positionTeamProp == 1) {
                driveToPositionController.setTargetPosition(targetXTeamPropCloseToBackdrop, targetYTeamProp - 300, 90.0).waitUntilPositionIsReached(2000);
            } else if (!startCloseToBackdrop && positionTeamProp == 2) {
                driveToPositionController.setTargetPosition(targetXTeamPropFarFromBackdrop - 100, targetYTeamProp, 0.0).waitUntilPositionIsReached(2000);
            }

            double targetY = 572;
            if ((positionTeamProp == 0 && redAlliance) || (positionTeamProp == 2 && !redAlliance)) {
                targetY -= 152;
            } else if ((positionTeamProp == 2 && redAlliance) || (positionTeamProp == 0 && !redAlliance)) {
                targetY += 152;
            }

            if ((scorePixelAtLeftBackdropPosition && redAlliance) || (!scorePixelAtLeftBackdropPosition && !redAlliance)) {
                targetY += 76;
            }

            if (!startCloseToBackdrop) {
                driveToPositionController.setTargetPosition(targetXTeamPropFarFromBackdrop, 1300.0, 0.0).waitUntilPositionIsReached(2000);
                driveToPositionController.setTargetPosition(targetXTeamPropFarFromBackdrop, 1300.0, 90.0).waitUntilPositionIsReached(2000);
                driveToPositionController.setTargetPosition(2400.0, 1300.0, 90.0).waitUntilPositionIsReached(2000);
                driveToPositionController.setTargetPosition(2400.0, 1300.0, 0.0).waitUntilPositionIsReached(1000);
                driveToPositionController.setTargetPosition(2400.0, targetY, 0.0).waitUntilPositionIsReached(2000);
            }
            driveToPositionController.setTargetPosition(2400.0, targetY, 90.0).waitUntilPositionIsReached(3000);
            elevator.setTargetElevationInmm(170);
            driveToPositionController.setTargetPosition(2750.0, targetY, 90.0).waitUntilPositionIsReached(4000);
            driveToPositionController.deactivate();
            Robot.drivetrain.driveWithoutAnyEnhancements(0.3, 0, 0).doDrive();
            Thread.sleep(700);
            Robot.drivetrain.stop();

            outputAndArm.releasePixel();
            Thread.sleep(1000);
            driveToPositionController.setTargetPosition(2600.0, targetY, 90.0);
            driveToPositionController.activate();
            driveToPositionController.waitUntilPositionIsReached(2000);
            //TODO: Make this better
            Thread.sleep(1000);
            elevator.setTargetElevationInmm(-40);
            outputAndArm.retractServo();

            //TODO: watch out to don't descore the pixel on the spike mark
            if (startCloseToBackdrop && !alliancePartnerIsScoringAtBackdrop) {
                driveToPositionController.setTargetPosition(2500.0, 750.0, 90.0);
                driveToPositionController.waitUntilPositionIsReached(2000);
                driveToPositionController.setTargetPosition(2500.0, 750.0, 135.0).waitUntilPositionIsReached(2000);
                elevator.deactivate();
                outputAndArm.deactivate();
                driveToPositionController.setTargetPosition(2000.0, 1250.0, 135.0).waitUntilPositionIsReached(2000);
                driveToPositionController.setTargetPosition(2000.0, 1250.0, 90.0).waitUntilPositionIsReached(2000);
                rackAndPinion.activateIntake();
                driveToPositionController.setTargetPosition(-200.0, 1250.0, 90.0).waitUntilPositionIsReached(4000);
                Thread.sleep(1000);
                driveToPositionController.setTargetPosition(0.0, 1250.0, 90.0).waitUntilPositionIsReached(2000);
                driveToPositionController.setTargetPosition(-200.0, 1250.0, 90.0).waitUntilPositionIsReached(2000);
                Thread.sleep(1000);
                rackAndPinion.deactivateIntake();
                driveToPositionController.setTargetPosition(2400.0, 1250.0, 90.0).waitUntilPositionIsReached(2000);
                driveToPositionController.setTargetPosition(2400.0, 1250.0, -65.0).waitUntilPositionIsReached(2000);
                driveToPositionController.setTargetPosition(2700.0, 1000.0, -65.0).waitUntilPositionIsReached(2000);
                rackAndPinion.activateIntakeReversed();
                driveToPositionController.setTargetPosition(2400.0, 1300.0, -65.0).waitUntilPositionIsReached(4000);
                rackAndPinion.deactivateIntake();
                driveToPositionController.setTargetPosition(2600.0, 1100.0, -65.0).waitUntilPositionIsReached(2000);
            } else if (startCloseToBackdrop) {
                driveToPositionController.setTargetPosition(2500.0, targetY, 90.0).waitUntilPositionIsReached(8000);
                driveToPositionController.setTargetPosition(2500.0, 100.0, 90.0).waitUntilPositionIsReached(2000);
                elevator.deactivate();
                outputAndArm.deactivate();
            }

            // Collect Pixel of Alliance Partner
            if (!startCloseToBackdrop && !alliancePartnerIsScoringOnSpikeMark) {
                driveToPositionController.setTargetPosition(2600.0, -50.0, 90.0).waitUntilPositionIsReached(2000);
                driveToPositionController.setTargetPosition(2100.0, -50.0, 90.0).waitUntilPositionIsReached(2000);
                rackAndPinion.setPower(0.6);
                drivetrain.driveWithoutAnyEnhancements(0.3, 0, 0).doDrive();
                Thread.sleep(1000);
                rackAndPinion.deactivateIntake();
                drivetrain.stop();

                if (positionTeamProp == 0) {
                    driveToPositionController.setTargetPosition(2300.0, targetYTeamProp - 100, 90.0).waitUntilPositionIsReached(2000);
                    driveToPositionController.setTargetPosition(targetXTeamPropCloseToBackdrop - 380, targetYTeamProp - 100, 90.0).waitUntilPositionIsReached(2000);
                    rackAndPinion.setPower(-0.4);
                    driveToPositionController.setTargetPosition(2600.0, targetYTeamProp - 100, 90.0).waitUntilPositionIsReached(2000);
                    rackAndPinion.deactivateIntake();
                } else if (positionTeamProp == 1) {
                    driveToPositionController.setTargetPosition(2300.0, targetYTeamProp + 200, 90.0).waitUntilPositionIsReached(2000);
                    driveToPositionController.setTargetPosition(targetXTeamPropCloseToBackdrop - 220, targetYTeamProp + 200, 90.0).waitUntilPositionIsReached(2000);
                    rackAndPinion.setPower(-0.4);
                    driveToPositionController.setTargetPosition(2600.0, targetYTeamProp + 200, 90.0).waitUntilPositionIsReached(2000);
                    rackAndPinion.deactivateIntake();
                } else {
                    driveToPositionController.setTargetPosition(2300.0, targetYTeamProp, 90.0).waitUntilPositionIsReached(2000);
                    driveToPositionController.setTargetPosition(targetXTeamPropCloseToBackdrop + 90, targetYTeamProp, 90.0).waitUntilPositionIsReached(2000);
                    rackAndPinion.setPower(-0.4);
                    driveToPositionController.setTargetPosition(2600.0, targetYTeamProp, 90.0).waitUntilPositionIsReached(2000);
                    rackAndPinion.deactivateIntake();
                }

                driveToPositionController.setTargetPosition(2650.0, targetY, 90.0).waitUntilPositionIsReached(2000);
            }

            driveToPositionController.deactivate();
        }
        stopRobot();
    }
}*/
