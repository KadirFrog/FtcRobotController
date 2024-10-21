package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.math.Position;

public class LastPositionStorage {

    static boolean dataStored = false;
    static long storageTime = 0;
    final static long dataValidDurationMillis = 60000;

    static Position lastPosition = new Position(0.0, 0.0, 0.0);
    static int lastExtraDegrees = 0;

    public static void storeData(Position position, int extraDegrees) {
        dataStored = true;
        lastPosition = position;
        lastExtraDegrees = extraDegrees;
        storageTime = System.currentTimeMillis();
    }

    public static boolean validDataAvailable() {
        return storageTime + dataValidDurationMillis > System.currentTimeMillis();
    }

    public static Position getLastPosition() {
        return lastPosition;
    }
    public static int getLastExtraDegrees() {
        return lastExtraDegrees;
    }
}
