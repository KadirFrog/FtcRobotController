package org.firstinspires.ftc.teamcode.hardware.sensor.camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

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
