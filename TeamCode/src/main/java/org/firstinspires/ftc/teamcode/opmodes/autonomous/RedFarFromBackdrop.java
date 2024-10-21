/*package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.util.Size;

import org.firstinspires.ftc.teamcode.MatchData;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.sensor.camera.TeamPropDetection;
import org.firstinspires.ftc.teamcode.math.Position;
import org.firstinspires.ftc.vision.VisionPortal;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RedFarFromBackdrop", group = "!Important")
@SuppressWarnings("ConstantConditions")
public class RedFarFromBackdrop extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        MatchData.redAlliance = true;
        init(hardwareMap);

        double targetXTeamPropFarFromBackdrop = 500;
        trackPosition.setPosition(new Position(600.0, 0.0, 0.0));
        TeamPropDetection teamPropDetection = new TeamPropDetection();

        visionPortal = new VisionPortal.Builder().setCamera(front_webcam).addProcessors(drawFrame, teamPropDetection).setCameraResolution(new Size(640, 480)).setAutoStopLiveView(true).build();

        driveToPositionController.setMaxDeviation(20, 20, 5);
        driveToPositionController.activate();

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

            double targetYTeamProp = 750;
            if (positionTeamProp == 0) {
                driveToPositionController.setTargetPosition(targetXTeamPropFarFromBackdrop, targetYTeamProp, 0.0).waitUntilPositionIsReached(2000);
                driveToPositionController.setTargetPosition(targetXTeamPropFarFromBackdrop, targetYTeamProp, 90.0);
            } else if (positionTeamProp == 1) {
                driveToPositionController.setTargetPosition(targetXTeamPropFarFromBackdrop, 1250.0, 0.0);
            } else {
                driveToPositionController.setTargetPosition(targetXTeamPropFarFromBackdrop, targetYTeamProp, 0.0).waitUntilPositionIsReached(2000);
                driveToPositionController.setTargetPosition(targetXTeamPropFarFromBackdrop, targetYTeamProp, -90.0);
            }

            driveToPositionController.waitUntilPositionIsReached(2000);
            driveToPositionController.deactivate();
            rackAndPinion.setPower(-0.3);
            Thread.sleep(500);
            rackAndPinion.deactivateIntake();
            pixelsInRobot--;
            driveToPositionController.activate();

            if (positionTeamProp == 0) {
                driveToPositionController.setTargetPosition(targetXTeamPropFarFromBackdrop + 100, targetYTeamProp, 0.0).waitUntilPositionIsReached(2000);
            } else if (positionTeamProp == 2) {
                driveToPositionController.setTargetPosition(targetXTeamPropFarFromBackdrop - 100, targetYTeamProp, 0.0).waitUntilPositionIsReached(2000);
            }

            double targetY = 650;
            if (positionTeamProp == 0) {
                targetY = 498;
            } else if (positionTeamProp == 2) {
                targetY = 802;
            }

            driveToPositionController.setTargetPosition(600.0, 1300.0, 0.0).waitUntilPositionIsReached(2000);
            driveToPositionController.setTargetPosition(600.0, 1300.0, 90.0).waitUntilPositionIsReached(2000);
            driveToPositionController.setTargetPosition(2400.0, 1300.0, 90.0).waitUntilPositionIsReached(2000);
            driveToPositionController.setTargetPosition(2400.0, 1300.0, 0.0).waitUntilPositionIsReached(1000);
            driveToPositionController.setTargetPosition(2400.0, targetY, 0.0).waitUntilPositionIsReached(2000);

            driveToPositionController.setTargetPosition(2400.0, targetY, 90.0).waitUntilPositionIsReached(3000);
            elevator.setTargetElevationInmm(150);
            driveToPositionController.setTargetPosition(2750.0, targetY, 90.0).waitUntilPositionIsReached(4000);
            driveToPositionController.deactivate();
            Robot.drivetrain.driveWithoutAnyEnhancements(0.3, 0, 0).doDrive();
            Thread.sleep(700);
            Robot.drivetrain.stop();

            outputAndArm.releasePixel();
            Thread.sleep(1000);
            driveToPositionController.setTargetPosition(2500.0, targetY, 90.0);
            driveToPositionController.activate();
            driveToPositionController.waitUntilPositionIsReached(2000);
            elevator.setTargetElevationInmm(-40);
            outputAndArm.retractServo();

            driveToPositionController.deactivate();
        }
        stopRobot();
    }
}*/
