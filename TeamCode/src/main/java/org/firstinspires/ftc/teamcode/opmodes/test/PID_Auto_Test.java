package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.math.Position;

@Autonomous(name = "test PID")
public class PID_Auto_Test extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap);
        Position startingPosition = new Position(0.0, 0.0, 0.0);
        driveToPositionController.setMaxDeviation(10, 10, 10);
        driveToPositionController.activate();
        addExecutor(() -> {
            while (!isStopRequested()) {
                trackPosition.printTelemetry(telemetry);
                telemetry.update();
            }
            state = State.SHOULD_STOP;
        });
        pixelsInRobot = 2;
        state = State.INITIALIZED;
        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();
        driveToPositionController.setTargetPosition(100.0, 0.0, 0.0);

        driveToPositionController.waitUntilPositionIsReached(4000);
        driveToPositionController.setTargetPosition(100.0, 100.0, 0.0);
        driveToPositionController.waitUntilPositionIsReached(4000);
        stopRobot();
    }


}
