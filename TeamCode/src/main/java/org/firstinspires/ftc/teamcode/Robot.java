package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.DriveToPositionController;
import org.firstinspires.ftc.teamcode.control.ErrorDetection;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Elevator;
import org.firstinspires.ftc.teamcode.hardware.RackAndPinion;
import org.firstinspires.ftc.teamcode.hardware.sensor.TrackPosition;
import org.firstinspires.ftc.teamcode.hardware.sensor.IMU;
import org.firstinspires.ftc.teamcode.hardware.sensor.camera.DrawFrame;
import org.firstinspires.ftc.teamcode.input.InputManager;
import org.firstinspires.ftc.teamcode.math.Vec3d;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@SuppressWarnings("unused")
public abstract class Robot extends LinearOpMode {
    @SuppressWarnings("SpellCheckingInspection")
    //Vuforia key: AUVEa4X/////AAABmShEK2ul9E26pGpzpiAhMaRKUxT5uUKR/Sn/nWoTX5Cb0fIFWiZXPhJbwXZCWNrhuoArAhL7klDyMoUJ1t5Q2YD8ob7BTXM887M2PVPG5tO1ZFO1nB2zXrFHUn17W3lW/+Uwn6zOn++PGdhgXktzjp+A9o4EZI8TRKqnvgD6PDPwRbTE0GUrrPvBjCm+WNXzNMEncTzDj7jmkqDgYLTio//r7mskweZJveK2H6C7elt2wbQPVkZym5qgkoHeBw/PZNZ+mGN4BLRF/KQCaPS1KHYD15NLEykt3t70KTGdkPh653Z9eXpOBlPanL3zUscBAWSBDHFbO+rdlFmk9G2B4d6OvdMjEbzHhfGmfFkTtPN5

    public static TrackPosition trackPosition;

    public static DriveToPositionController driveToPositionController;
    public static ErrorDetection errorDetection;
    public static Drivetrain drivetrain;
    public static Elevator elevator;
    public static RackAndPinion rackAndPinion;

    public static InputManager inputManager;

    public static IMU imu1;
    public static IMU imu2;

    //Tensorflow
    public static WebcamName front_webcam;
    //public static WebcamName rear_webcam;
    public static CameraName switchableCamera;
    public static DrawFrame drawFrame;

    public static VisionPortal visionPortal;
    public static AprilTagProcessor aprilTag;


    public static LynxModule controlHub;
    public static LynxModule expansionHub;

    static ExecutorService executor;

    public enum State {
        STARTED,
        INITIALIZED,
        AUTONOMOUS,
        TELEOP,
        SHOULD_STOP,
    }

    public static State state = State.STARTED;

    public static int pixelsInRobot = 0;

    public static void init(HardwareMap hardwareMap) {
        state = State.STARTED;

        System.loadLibrary("track_position");

        driveToPositionController = new DriveToPositionController();
        errorDetection = new ErrorDetection();
        //deadWheelOdometry = new DeadWheelOdometry(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        elevator = new Elevator(hardwareMap);
        rackAndPinion = new RackAndPinion(hardwareMap);

        imu1 = new IMU(hardwareMap.get(BNO055IMU.class, "imu_1"), Vec3d.Vec3dRemapOrder.ZYX_XYZ);
        //imu2 = new IMU(hardwareMap.get(BNO055IMU.class, "imu2"), Vec3d.Vec3dRemapOrder.ZYX_XYZ);

        front_webcam = hardwareMap.get(WebcamName.class, "front_webcam");
        //rear_webcam = hardwareMap.get(WebcamName.class, "rear_webcam");
        switchableCamera = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(front_webcam);
        drawFrame = new DrawFrame();
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(3);

        executor = Executors.newFixedThreadPool(10);
        //addExecutor(trackPosition.backgroundTask);
        //addExecutor(errorDetection.resetTimerIfOpModeIsStarted);
        //addExecutor(intake.backgroundTask);

        // Bulk reading
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        controlHub = hubs.get(0);
        expansionHub = hubs.get(1);
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        trackPosition = new TrackPosition(hardwareMap);

        inputManager = new InputManager();
    }

    public void stopRobot() {
        trackPosition.stopTrackingAndStoreData();
        driveToPositionController.deactivate();
        drivetrain.stop();
        elevator.stop();
        visionPortal.close();
        errorDetection.closeErrorLogWriter();

        try {
            executor.shutdown();
            Thread.sleep(50);
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            executor.shutdownNow();
        }
        stop();
    }

    public static void addExecutor(Runnable runnable) {
        executor.execute(runnable);
    }
}