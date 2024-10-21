package org.firstinspires.ftc.teamcode.control;

import android.os.Environment;

import org.firstinspires.ftc.teamcode.Robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/*public class PathOptimizer {
    private static final File correctionXFile = new File(Environment.getExternalStorageDirectory().getPath() + "/CorrectionX.txt");
    double lastX = 0;
    double correctionX = 0;

    public void pathOptimizer() throws InterruptedException {
        for (int i = 0; i < 3; i++) {
            while (Robot.trackPosition.getRobotPositionX() < 1030 && Robot.trackPosition.getRobotPositionX() > 970) {
                Robot.drivetrain.driveConstantVelocity(1, 0, 0);
                while (Robot.trackPosition.getRobotPositionX() <= 1000) ;
                lastX = Robot.trackPosition.getRobotPositionX();
                Robot.drivetrain.stop();
                Thread.sleep(1000);
                correctionX = lastX - Robot.trackPosition.getRobotPositionX();

                Robot.drivetrain.driveConstantVelocity(-1,0, 0);
                Thread.sleep(2000);
                Robot.drivetrain.stop();

                Robot.trackPosition.setRobotPosition(0, Robot.trackPosition.getRobotPositionY(), Robot.trackPosition.getRobotRotation());
            }
        }
        if (correctionXFile.delete()) {
            System.out.println("Deleted the file: " + correctionXFile.getName());
        } else {
            System.out.println("Failed to delete the file.");
        }

        try {
            if (correctionXFile.createNewFile()) {
                System.out.println("Created the file:  " + correctionXFile.getName());
            } else {
                System.out.println("Failed to create the file.");
            }
        } catch (final IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }

        try {
            final FileWriter turnWriter = new FileWriter(correctionXFile);
            turnWriter.write(Double.toString(correctionX));
            turnWriter.close();
            System.out.println("Successfully wrote to the file.");
        } catch (final IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }
    }
}*/

/*
package WRO.initialization;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import WRO.functions.Motors;
import WRO.functions.Sensors;
import WRO.functions.WriteFileToVariable;
import lejos.utility.Delay;

class InitTurn {
    private static final File correctionXFile = new File("/home/robot/utilities/initTxt/Turn.txt");
    private static int correction = WriteFileToVariable.correction;
    private static int exec = 0;
    private static long time = 0;
    private static final double direction = 50;

    public static void main(final String[] args) {
        InitEverything.initEverything();

        for (int i = 0; i < 3; i++) {
            Sensors.setZeroGyro();
            while (Sensors.angle() < 88 || Sensors.angle() > 92) {
                Sensors.setZeroGyro();
                Delay.msDelay(500);
                time = System.currentTimeMillis();
                Motors.driveCm(400, direction, Motors.angleToCm(90-correction, direction), "brake");
                time = System.currentTimeMillis() - time;
                System.out.println("drive time:" + time);
                System.out.println("executions:" + exec);
                System.out.println("angle: " + Sensors.angle());
                Delay.msDelay(1000);
                correction = Sensors.angle() - 90 + correction;
                System.out.println("correction:" + correction);
            }
        }

        if (correctionXFile.delete()) {
            System.out.println("Deleted the file: " + correctionXFile.getName());
        } else {
            System.out.println("Failed to delete the file.");
        }

        try {
            if (correctionXFile.createNewFile()) {
                System.out.println("Created the file:  " + correctionXFile.getName());
            } else {
                System.out.println("Failed to create the file.");
            }
        } catch (final IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }

        try {
            final FileWriter turnWriter = new FileWriter(correctionXFile);
            turnWriter.write(Integer.toString(correction));
            turnWriter.close();
            System.out.println("Successfully wrote to the file.");
        } catch (final IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }
        System.exit(0);
    }
}

 */
