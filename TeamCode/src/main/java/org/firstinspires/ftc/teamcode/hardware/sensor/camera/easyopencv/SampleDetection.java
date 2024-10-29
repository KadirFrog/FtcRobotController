package org.firstinspires.ftc.teamcode.hardware.sensor.camera.easyopencv;

import android.graphics.Canvas;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleDetection extends OpenCvPipeline {
    public class PictureNormalization {
        public Mat normalizeValue(Mat hsv) {
            List<Mat> channels = new ArrayList<>();
            Core.split(hsv, channels);
            Mat valueChannel = channels.get(2);

            double averageValue = Core.mean(valueChannel).val[0];
            valueChannel.convertTo(valueChannel, -1, 128 / averageValue, 0);
            valueChannel.copyTo(channels.get(2));

            Core.merge(channels, hsv);
            return hsv;
        }
    }
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



    public void setColor(int colorCode) { // 0 -> red, 1 -> blue, 2 -> yellow
        if (colorCode < 3 && colorCode > -1) {
            SampleColor = colorCode;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
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
        List<MatOfPoint> polygons = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            MatOfPoint2f polygon2f = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, polygon2f, 5, true); // Adjust epsilon value as needed
            MatOfPoint polygon = new MatOfPoint(polygon2f.toArray());
            polygons.add(polygon);
        }

        for (MatOfPoint polygon : polygons) {
            Imgproc.polylines(input, polygons, true, new Scalar(0, 255, 0), 2);
        }

        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
