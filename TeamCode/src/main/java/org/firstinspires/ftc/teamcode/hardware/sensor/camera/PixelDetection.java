package org.firstinspires.ftc.teamcode.hardware.sensor.camera;

import android.graphics.Canvas;
import android.graphics.Color;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@SuppressWarnings("unused")
public class PixelDetection implements VisionProcessor {

    final Scalar blackBottomHSV = new Scalar(0, 0, 0);
    final Scalar blackUpperHSV = new Scalar(0, 255, 30);
    Map<Point, Integer> pixelPositionAndColor = new HashMap<>();

    PictureNormalization pictureNormalization = new PictureNormalization();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        hsv = pictureNormalization.normalizeValue(hsv);

        Mat thresh = new Mat();
        Core.inRange(hsv, blackBottomHSV, blackUpperHSV, thresh);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<MatOfPoint> innerPixelContours = new ArrayList<>();

        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) < 100) {
                MatOfPoint2f approxCurveContour = new MatOfPoint2f();
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
                Imgproc.approxPolyDP(contour2f, approxCurveContour, epsilon, true);

                if (approxCurveContour.total() == 6) {
                    innerPixelContours.add(contour);
                }
            }
        }

        for (MatOfPoint contour : innerPixelContours) {
            Moments moments = Imgproc.moments(contour);

            Point topLeft = new Point(moments.get_m10()  / moments.get_m00() - moments.get_m10() / 2, moments.get_m01() / moments.get_m00() - moments.get_m01() / 2);
            Point bottomRight = new Point(moments.get_m10()  / moments.get_m00() + moments.get_m10() / 2, moments.get_m01() / moments.get_m00() + moments.get_m01() / 2);
            Point center = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());

            hsv.submat(new Rect(topLeft, bottomRight));
            Mat pixel = new Mat();
            Core.inRange(hsv, blackBottomHSV, blackUpperHSV, pixel);
            Core.bitwise_not(pixel, pixel);
            Core.bitwise_and(hsv, hsv, pixel, pixel);
            Scalar average = Core.mean(pixel);

            if (average.val[1] > 100 && average.val[2] < 200) {
                pixelPositionAndColor.put(center, Color.WHITE);
            } else if (average.val[0] > 20 && average.val[0] < 50) {
                pixelPositionAndColor.put(center, Color.YELLOW);
            } else if (average.val[0] > 70 && average.val[0] < 100) {
                pixelPositionAndColor.put(center, Color.GREEN);
            } else {
                pixelPositionAndColor.put(center, Color.MAGENTA);
            }
        }


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Map<Point, Integer> getPixelPositionsAndColor() {
        return pixelPositionAndColor;
    }
}