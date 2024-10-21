package org.firstinspires.ftc.teamcode.opmodes.dashboard;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public static int BOTTOM_H_VALUE = 90;
    public static int UPPER_H_VALUE = 120;
    public static int BOTTOM_S_VALUE = 80;
    public static int UPPER_S_VALUE = 255;
    public static int BOTTOM_V_VALUE = 100;
    public static int UPPER_V_VALUE = 255;
    public static boolean BETTER_PICTURE = true;

    public double largestArea = -1;
    public double secondLargestArea = -1;
    public double thirdLargestArea = -1;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        if (BETTER_PICTURE) {
            List<Mat> channels = new ArrayList<>();
            Core.split(hsv, channels);
            Mat valueChannel = channels.get(2);
            double averageValue = Core.mean(valueChannel).val[0];
            valueChannel.convertTo(valueChannel, -1, 128 / averageValue, 0);
            valueChannel.copyTo(channels.get(2));
            Core.merge(channels, hsv);
        }

        Mat thresh = new Mat();

        if (BOTTOM_H_VALUE > UPPER_H_VALUE) {
            Mat thresh1 = new Mat();
            Mat thresh2 = new Mat();
            Core.inRange(hsv, new Scalar(0, BOTTOM_S_VALUE, BOTTOM_V_VALUE), new Scalar(UPPER_H_VALUE, UPPER_S_VALUE, UPPER_V_VALUE), thresh1);
            Core.inRange(hsv, new Scalar(BOTTOM_H_VALUE, BOTTOM_S_VALUE, BOTTOM_V_VALUE), new Scalar(180, UPPER_S_VALUE, UPPER_V_VALUE), thresh2);
            Core.bitwise_or(thresh1, thresh2, thresh);
        } else {
            Scalar lowHSV = new Scalar(BOTTOM_H_VALUE, BOTTOM_S_VALUE, BOTTOM_V_VALUE);
            Scalar highHSV = new Scalar(UPPER_H_VALUE, UPPER_S_VALUE, UPPER_V_VALUE);
            Core.inRange(hsv, lowHSV, highHSV, thresh);
        }
        Mat masked = new Mat();
        Core.bitwise_and(hsv, hsv, masked, thresh);
        Mat RGB = new Mat();
        Imgproc.cvtColor(masked, RGB, Imgproc.COLOR_HSV2RGB);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        largestArea = -1;
        secondLargestArea = -1;
        thirdLargestArea = -1;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > thirdLargestArea) {
                if (area > secondLargestArea) {
                    thirdLargestArea = secondLargestArea;

                    if (area > largestArea) {
                        secondLargestArea = largestArea;

                        largestArea = area;

                    } else {
                        secondLargestArea = area;
                    }
                } else {
                    thirdLargestArea = area;
                }
            }
        }

        Bitmap b = Bitmap.createBitmap(RGB.width(), RGB.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(RGB, b);
        lastFrame.set(b);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // do nothing
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("largest area", largestArea);
        telemetry.addData("team prop their", largestArea > 3500);
        telemetry.addData("second largest area", secondLargestArea);
        telemetry.addData("third largest area", thirdLargestArea);
    }
}