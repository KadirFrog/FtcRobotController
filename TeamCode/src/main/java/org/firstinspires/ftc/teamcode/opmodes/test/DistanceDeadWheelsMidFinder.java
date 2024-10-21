package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@SuppressWarnings({"unused", "StatementWithEmptyBody", "BusyWait"})
@Autonomous(name = "distance dead wheels mid finder", group = "Init")
public class DistanceDeadWheelsMidFinder extends Robot {
    double targetRotation = 0;

    double ENCODER_TICKS_PER_ROTATION = 8192;
    double WHEEL_DIAMETER_MM = 60;
    int encoderTicksCorrectedForward = 0;
    int encoderTicksCorrectedForwardSum = 0;
    int encoderTicksCorrectedSidewards = 0;
    int encoderTicksCorrectedSidewardsSum = 0;
    int loops = 0;
    double distanceEncoderSidewardsMid = 0;
    double distanceEncoderForwardMid = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);

        state = State.INITIALIZED;

        waitForStart();
        state = State.TELEOP;

        if (opModeIsActive()) {
            while (!gamepad1.b && state != State.SHOULD_STOP) {
                loops++;
                double rotationBeforeMove = trackPosition.getRotation();
                double forwardMmBeforeMove = trackPosition.getForwardMm();
                double sidewardsMmBeforeMove = trackPosition.getSidewardsMm();
                targetRotation = trackPosition.getRotation() + 90;
                drivetrain.driveWithoutAnyEnhancements(0, 0, 0.3);
                drivetrain.doDrive();
                while (trackPosition.getRotation() < targetRotation) ;
                drivetrain.stop();

                Thread.sleep(2000);
                encoderTicksCorrectedForward = (int) (((trackPosition.getForwardMm() - forwardMmBeforeMove) * ENCODER_TICKS_PER_ROTATION / (WHEEL_DIAMETER_MM * Math.PI)) / ((trackPosition.getRotation() - rotationBeforeMove) / 360));
                encoderTicksCorrectedForwardSum += encoderTicksCorrectedForward;
                encoderTicksCorrectedSidewards = (int) (((trackPosition.getSidewardsMm() - sidewardsMmBeforeMove) * ENCODER_TICKS_PER_ROTATION / (WHEEL_DIAMETER_MM * Math.PI)) / ((trackPosition.getRotation() - rotationBeforeMove) / 360));
                encoderTicksCorrectedSidewardsSum += encoderTicksCorrectedSidewards;
                telemetry.addData("Encoder Ticks corrected forward", encoderTicksCorrectedForward);
                telemetry.addData("Encoder Ticks corrected sidewards_speed", encoderTicksCorrectedSidewards);
                telemetry.update();
            }
            encoderTicksCorrectedForward = encoderTicksCorrectedForwardSum / loops;
            encoderTicksCorrectedSidewards = encoderTicksCorrectedSidewardsSum / loops;
            distanceEncoderForwardMid = (encoderTicksCorrectedForward / (2 * Math.PI)) / (ENCODER_TICKS_PER_ROTATION / (WHEEL_DIAMETER_MM * Math.PI));
            distanceEncoderSidewardsMid = (encoderTicksCorrectedSidewards / (2 * Math.PI)) / (ENCODER_TICKS_PER_ROTATION / (WHEEL_DIAMETER_MM * Math.PI));
            telemetry.addData("Distance encoder forward mid", distanceEncoderForwardMid);
            telemetry.addData("Distance encoder sidewards_speed mid", distanceEncoderSidewardsMid);
            telemetry.addLine("Press a if you wrote down the value");
            telemetry.update();

            while (!gamepad1.a && state != State.SHOULD_STOP) ;
        }
        stopRobot();
    }
}
