package frc.robot.util;


import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

public class Gyro {
    // Angle in degrees that designates a significant tilt.
    private static double degreesSignificance = 2.0;
    // Flattens a Rotation3d into a yaw, unless the tilt is above a threshold
    public static Rotation3d flatten(Rotation3d rotation3d){
        // From https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles -> Quaternions to angles Conversion
        double qw = rotation3d.getQuaternion().getW();
        double qx = rotation3d.getQuaternion().getX();
        double qy = rotation3d.getQuaternion().getY();
        double qz = rotation3d.getQuaternion().getZ();

        double roll = Math.atan2(2*(qw * qx + qy * qz),1 - 2 * (qx * qx + qy * qy));
        double pitch = -Math.PI/2 + 2 * Math.atan2(Math.sqrt(1 + 2 * (qw * qy - qx * qz)), Math.sqrt(1 - 2 * (qw * qy - qx * qz)));
        double yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
        

    }

}
