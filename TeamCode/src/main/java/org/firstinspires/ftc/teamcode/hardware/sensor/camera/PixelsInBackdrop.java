package org.firstinspires.ftc.teamcode.hardware.sensor.camera;

import android.graphics.Point;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Map;

public class PixelsInBackdrop {
    ArrayList<ArrayList<Integer>> colorOfPixelsInBackdrop = new ArrayList<>();
    public void setPixels (ArrayList<AprilTagDetection> aprilTagDetections, Map<Point, Integer> pixel) {
        colorOfPixelsInBackdrop.clear();
        pixel.forEach((center, color) -> {
            if (center.x < aprilTagDetections.get(0).center.x) {
                // later
            }
        });
    }
}
