package org.firstinspires.ftc.teamcode.hardware.sensor.camera;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class SampleDetection implements VisionProcessor {
    PictureNormalization pictureNormalization = new PictureNormalization();
    int SampleColor;
    Scalar lowHSV0, lowHSV1, lowHSV2, highHSV0, highHSV1, highHSV2;

    public SampleDetection() {
        lowHSV0 = new Scalar(0, 80, 50);
        highHSV0 = new Scalar(15, 255, 255);
        lowHSV1 = new Scalar(165, lowHSV0.val[1], lowHSV0.val[2]);
        highHSV1 = new Scalar(180, highHSV0.val[1], highHSV0.val[2]);
        lowHSV2 = new Scalar(35, lowHSV0.val[1], lowHSV0.val[2]);
        highHSV2 = new Scalar(65, highHSV0.val[1], highHSV0.val[2]);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    public void setColor(int colorCode) { // 0 -> red, 1 -> blue, 2 -> yellow
        if (colorCode < 3 && colorCode > -1) {
            SampleColor = colorCode;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        hsv = pictureNormalization.normalizeValue(hsv);

        Mat finalThresh = new Mat();
        if (SampleColor == 0) {
            Core.inRange(hsv, lowHSV0, highHSV0, finalThresh);
        } else if (SampleColor == 1) {
            Core.inRange(hsv, lowHSV1, highHSV1, finalThresh);
        } else {
            Core.inRange(hsv, lowHSV2, highHSV2, finalThresh);
        }
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(finalThresh, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<MatOfPoint> longContours = new ArrayList<>();
        for (MatOfPoint contour1 : contours) {
            MatOfPoint2f contour = new MatOfPoint2f(contour1.toArray());
            if (Imgproc.arcLength(contour, true) > 100) {
                longContours.add(contour1);
            }
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
