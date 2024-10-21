package org.firstinspires.ftc.teamcode.control;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

@SuppressWarnings({"unused", "ResultOfMethodCallIgnored", "StatementWithEmptyBody"})
public class ErrorDetection {
    Map<String, Integer> wrongVelocityCounts = new HashMap<>();
    Map<String, Boolean> motorsOnFire = new HashMap<>();
    long startTime;
    final File errorLogFile = new File(Environment.getExternalStorageDirectory().getPath() + "/ErrorLog.txt");
    final FileWriter errorLogWriter;

    public ErrorDetection() {
        startTime = System.currentTimeMillis();
        errorLogFile.delete();
        try {
            errorLogFile.createNewFile();
            errorLogWriter = new FileWriter(errorLogFile);
            errorLogWriter.write("-------------HARDWARE ISSUES-------------" + System.lineSeparator());
            errorLogWriter.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    // Checks if the motor encoder is unplugged or has other issues
    public boolean detectFire(String motorName, double currentValue, double targetValue, int maxWrongCurrentValues) {
        boolean motorOnFire = true;
        if (!Boolean.TRUE.equals(motorsOnFire.get(motorName))) {
            int wrongVelocityCount = 0;
            if (!wrongVelocityCounts.containsKey(motorName)) {
                wrongVelocityCounts.put(motorName, 0);
            } else {
                Integer currentCount = wrongVelocityCounts.get(motorName);
                if (currentCount != null) {
                    wrongVelocityCount = currentCount;
                }
            }

            motorOnFire = false;
            if (!motorsOnFire.containsKey(motorName)) {
                motorsOnFire.put(motorName, false);
            }

            if (targetValue != 0 && currentValue == 0) {
                wrongVelocityCount++;
                if (wrongVelocityCount == maxWrongCurrentValues) {
                    addErrorToErrorLog(motorName + ": Encoder damaged");
                    motorOnFire = true;
                    motorsOnFire.put(motorName, true);
                }
            } else if (targetValue != 0) {
                wrongVelocityCount = 0;
            }
            wrongVelocityCounts.put(motorName, wrongVelocityCount);
        }
        return motorOnFire;
    }

    public boolean detectFire(String motorName, double currentValue, double targetValue) {
        return detectFire(motorName, currentValue, targetValue, 10);
    }

    public boolean isMyMotorOnFire(String motorName) {
        return Boolean.TRUE.equals(motorsOnFire.get(motorName));
    }

    public Map<String, Boolean> getMotorsOnFire() {
        return motorsOnFire;
    }

    public Runnable resetTimerIfOpModeIsStarted = () -> {
        while (Robot.state != Robot.State.TELEOP && Robot.state != Robot.State.AUTONOMOUS);
        startTime= System.currentTimeMillis();
    };

    public void addErrorToErrorLog(String error) {
        long elapsedTime = System.currentTimeMillis() - startTime;
        double minutes = Math.floor((elapsedTime / (1000.0 * 60)) % 60);
        double seconds = Math.floor((elapsedTime / 1000.0) % 60);
        long milliseconds = elapsedTime % 1000;

        try {
            errorLogWriter.write(System.lineSeparator() + Robot.state.toString() + " %f:%f:%d %s" + minutes + seconds + milliseconds + error);
            errorLogWriter.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void closeErrorLogWriter() {
        try {
            errorLogWriter.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void printMotorStatus(Telemetry telemetry) {
        motorsOnFire.forEach((motor, onFire) -> {
            if (onFire) {
                telemetry.addData(motor, "Encoder is damaged");
            }
        });
    }
}
