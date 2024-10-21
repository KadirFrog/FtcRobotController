package org.firstinspires.ftc.teamcode.hardware.sensor.camera;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class DrawFrame implements VisionProcessor {
    Mat input = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        frame.copyTo(input);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Bitmap bitmap = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(input, bitmap);
        Bitmap scaledBitmap = Bitmap.createScaledBitmap(bitmap, (int)(640 * scaleBmpPxToCanvasPx), (int) (480 * scaleBmpPxToCanvasPx), true);
        canvas.drawBitmap(scaledBitmap, 0, 0, null);
    }

    public void drawContours (List<MatOfPoint> contours, int contourIdx, @NonNull Scalar color, int thickness) {
        Imgproc.drawContours(input, contours, contourIdx, color, thickness);
    }

    public void arrowedLine(@NonNull Point pt1, @NonNull Point pt2, @NonNull Scalar color, int thickness) {
        Imgproc.arrowedLine(input, pt1, pt2, color, thickness);
    }
}
