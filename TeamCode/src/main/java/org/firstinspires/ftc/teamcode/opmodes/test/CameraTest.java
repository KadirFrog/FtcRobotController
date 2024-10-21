package org.firstinspires.ftc.teamcode.opmodes.test;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.sensor.camera.TeamPropDetection;
import org.firstinspires.ftc.vision.VisionPortal;

@SuppressWarnings("unused")
@TeleOp(name = "CameraTest", group = "test")
public class CameraTest extends Robot {
    TeamPropDetection teamPropDetection = new TeamPropDetection();
    int positionTeamProp = 0;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        init(hardwareMap);

        visionPortal = new VisionPortal.Builder().setCamera(front_webcam).addProcessors(drawFrame, teamPropDetection).setCameraResolution(new Size(640, 480)).setAutoStopLiveView(true).build();
        state = State.INITIALIZED;

        while (opModeInInit() || opModeIsActive()) {
            positionTeamProp = teamPropDetection.getPositionTeamProp();
            telemetry.addData("Frame Count", visionPortal.getFps());
            telemetry.addData("Position Team Prop", positionTeamProp);
            telemetry.update();
        }

        stopRobot();
    }
}