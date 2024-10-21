package org.firstinspires.ftc.teamcode.opmodes.dashboard;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.vision.VisionPortal;

@SuppressWarnings("unused")
@TeleOp(name = ".Dashboard", group = "dashboard")
@Config
public class Dashboard extends Robot {
    double rotationOffsetRad = 0;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    final CameraStreamProcessor processor = new CameraStreamProcessor();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = dashboard.getTelemetry();

        init(hardwareMap);
        new VisionPortal.Builder().addProcessor(processor).setCamera(Robot.front_webcam).setCameraResolution(new Size(640, 480)).build();
        dashboard.startCameraStream(processor, 30);

        state = State.INITIALIZED;
        waitForStart();

        state = State.TELEOP;

        /*
        driveToPositionController.setMaxDeviation(20, 20, 10);
        driveToPositionController.activate();
        addExecutor(() -> {
            while (!isStopRequested()) {
                trackPosition.printTelemetry(telemetry);
                drivetrain.printStuff(telemetry);
                telemetry.update();
            }
            state = State.SHOULD_STOP;
        });

        drivetrain.useDeadWheelSupport(false);
        state = State.INITIALIZED;
        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();
        state = State.TELEOP;
        boolean count1, count2;
        count1 = false; count2 = false;
        if (opModeIsActive()) {
            boolean True = true;
            while (True && !isStopRequested()) {
            */
                /*
                if (gamepad1.dpad_down) {
                    drivetrain.driveWithoutAnyEnhancements(-150.0, 0.0, 0.0);
                }*/
        /*
            if (gamepad1.x && !count1) {
                driveToPositionController.setTargetPosition(null, null, 180.0);
                driveToPositionController.waitUntilPositionIsReached(4000);
                count1 = true;
            } else if (gamepad1.y && !count2){
                driveToPositionController.setTargetPosition(null, null, 0.0);
                driveToPositionController.waitUntilPositionIsReached(4000);
                count2 = true;
            } else if (gamepad1.b) {
                count1 = false; count2 = false;
            } else if (gamepad1.dpad_up){
                True = false;
                }
            }
        }
         */

        dashboard.stopCameraStream();
        stopRobot();
    }

}

