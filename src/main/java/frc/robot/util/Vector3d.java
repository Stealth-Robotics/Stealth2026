package frc.robot.util;

public class Vector3d {
    public double x, y, z;

    public Vector3d(double dx, double dy, double dz) {
        this.x = dx;
        this.y = dy;
        this.z = dz;
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }
}