package org.firstinspires.ftc.teamcode.math;

import androidx.annotation.NonNull;

public class Vec3d {

    public double x, y, z;

    public Vec3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    @NonNull
    public String toString() {
        return x + "/" + y + "/" + z;
    }

    public Vec3d remap(Vec3dRemapOrder remapOrder) {
        if(remapOrder == Vec3dRemapOrder.XYZ_XYZ) {
            return this;
        } else if (remapOrder == Vec3dRemapOrder.XZY_XYZ) {
            return new Vec3d(x, z, y);
        }else if (remapOrder == Vec3dRemapOrder.YXZ_XYZ) {
            return new Vec3d(y, x, z);
        }else if (remapOrder == Vec3dRemapOrder.YZX_XYZ) {
            return new Vec3d(y, z, x);
        }else if (remapOrder == Vec3dRemapOrder.ZYX_XYZ) {
            return new Vec3d(z, y, x);
        }else if (remapOrder == Vec3dRemapOrder.ZXY_XYZ) {
            return new Vec3d(z, x, y);
        }
        return this;
    }

    public enum Vec3dRemapOrder {
        XYZ_XYZ,
        XZY_XYZ,
        YXZ_XYZ,
        YZX_XYZ,
        ZYX_XYZ,
        ZXY_XYZ
    }


}
