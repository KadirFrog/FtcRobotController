package org.firstinspires.ftc.teamcode.hardware.sensor.camera;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.MatchData;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class TeamPropDetection implements VisionProcessor {

    final boolean redAlliance;
    final Scalar allianceColor;

    final Scalar lowHSV1;
    final Scalar highHSV1;

    Scalar lowHSV2;
    Scalar highHSV2;

    MatOfPoint largestContour = new MatOfPoint();
    MatOfPoint secondLargestContour = new MatOfPoint();

    int positionTeamProp = 0;
    boolean noTeamPropDetected = false;

    PictureNormalization pictureNormalization = new PictureNormalization();

    public TeamPropDetection() {
        this.redAlliance = MatchData.redAlliance;
        if (redAlliance) {
            allianceColor = new Scalar(255, 0, 0);
            lowHSV1  = new Scalar(0, 80, 50);
            highHSV1 = new Scalar(15, 255, 255);
            lowHSV2 = new Scalar(165, lowHSV1.val[1], lowHSV1.val[2]);
            highHSV2 = new Scalar(180, highHSV1.val[1], highHSV1.val[2]);
        } else {
            allianceColor = new Scalar(0, 0, 255);
            lowHSV1 = new Scalar(90, 80, 80);
            highHSV1 = new Scalar(120, 255, 255);
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        hsv = pictureNormalization.normalizeValue(hsv);

        Mat finalThresh = new Mat();
        if (redAlliance) {
            Mat thresh1 = new Mat();
            Mat thresh2 = new Mat();

            Core.inRange(hsv, lowHSV1, highHSV1, thresh1);
            Core.inRange(hsv, lowHSV2, highHSV2, thresh2);
            Core.bitwise_or(thresh1, thresh2, finalThresh);
        } else {
            Core.inRange(hsv, lowHSV1, highHSV1, finalThresh);
        }

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(finalThresh, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double largestArea = -1;
        double secondLargestArea = -1;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
                if (area > secondLargestArea) {
                    if (area > largestArea) {
                        secondLargestArea = largestArea;
                        secondLargestContour = largestContour;

                        largestArea = area;
                        largestContour = contour;

                    } else {
                        secondLargestArea = area;
                        secondLargestContour = contour;
                    }
                }
            }

        Point midLargestContour;
        Point midSecondLargestContour = new Point();
        MatOfPoint2f approxCurveLargestContour = new MatOfPoint2f();
        MatOfPoint2f approxCurveSecondLargestContour = new MatOfPoint2f();

        if (!largestContour.empty()) {
            noTeamPropDetected = false;

            MatOfPoint2f contour2f = new MatOfPoint2f(largestContour.toArray());
            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
            Imgproc.approxPolyDP(contour2f, approxCurveLargestContour, epsilon, true);

            Moments moments = Imgproc.moments(largestContour);
            midLargestContour = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());

            if (!secondLargestContour.empty()) {
                contour2f = new MatOfPoint2f(secondLargestContour.toArray());
                epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
                Imgproc.approxPolyDP(contour2f, approxCurveSecondLargestContour, epsilon, true);

                moments = Imgproc.moments(secondLargestContour);
                midSecondLargestContour = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());
            }

            Scalar largestContourColor = new Scalar(255, 255, 255);
            Scalar secondLargestContourColor = new Scalar(255, 255, 255);
            Scalar arrowColor = new Scalar(255, 255, 255);

            if (approxCurveLargestContour.total() > 5 && !largestContour.empty()) {
                if (Robot.visionPortal.getProcessorEnabled(Robot.drawFrame)) {
                    largestContourColor = allianceColor;
                }

                if (midLargestContour.x < midSecondLargestContour.x) {
                    positionTeamProp = 1;
                } else {
                    positionTeamProp = 2;
                }
            } else if (approxCurveSecondLargestContour.total() > 5 && !secondLargestContour.empty()) {
                if (Robot.visionPortal.getProcessorEnabled(Robot.drawFrame)) {
                    secondLargestContourColor = allianceColor;
                }

                if (midSecondLargestContour.x < midLargestContour.x) {
                    positionTeamProp = 1;
                } else {
                    positionTeamProp = 2;
                }
            } else {
                if (Robot.visionPortal.getProcessorEnabled(Robot.drawFrame)) {
                    arrowColor = allianceColor;
                }
                positionTeamProp = 0;
            }

            Robot.drawFrame.drawContours(Collections.singletonList(largestContour), -1, largestContourColor, 5);
            Robot.drawFrame.drawContours(Collections.singletonList(secondLargestContour), -1, secondLargestContourColor, 5);
            Robot.drawFrame.arrowedLine(new Point(100, 120), new Point(0, 120), arrowColor, 5);
        } else {
            noTeamPropDetected = true;
            // It will use the last detected position
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public int getPositionTeamProp() {
        if (noTeamPropDetected) {
            Robot.errorDetection.addErrorToErrorLog("no team prop detected");
        }
        return positionTeamProp;
    }
}