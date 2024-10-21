package org.firstinspires.ftc.teamcode.math;

import androidx.annotation.NonNull;

@SuppressWarnings("UnusedReturnValue")
public class Position {

    public Double x, y, rotation;

    public Position(Double x, Double y, Double rotation) {
        this.x = x;
        this.y = y;
        this.rotation = rotation;
    }

    public void set(Double x, Double y, Double rotation) {
        this.x = x;
        this.y = y;
        this.rotation = rotation;
    }

    public Position setX(Double x) {
        this.x = x;
        return this;
    }

    public Position setY(Double y) {
        this.y = y;
        return this;
    }

    public Position setRotation(Double rotation) {
        this.rotation = rotation;
        return this;
    }

    @NonNull
    public String toString() {
        return x + "/" + y + "/" + rotation;
    }
}
