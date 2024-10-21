package org.firstinspires.ftc.teamcode.math;

import androidx.annotation.NonNull;

@SuppressWarnings("unused")
public class Vec2d {

    public double x, y;

    public Vec2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void set(double x, double y) {
        this.x = x;
        this.y = y;
    }

    @NonNull
    public String toString() {
        return x + "/" + y;
    }

    public Vec2d remap(Vec2dRemapOrder remapOrder) {
        if(remapOrder == Vec2dRemapOrder.XY_XY) {
            return this;
        } else if (remapOrder == Vec2dRemapOrder.XY_YX) {
            return new Vec2d(y, x);
        }
        return this;
    }

    public enum Vec2dRemapOrder {
        XY_XY,
        XY_YX
    }
}
