package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class LimelightConstants {
    public static final int LIMELIGHT_DISABLED_THROTTLE = 100;

    public static final double VISION_XY_STDDEV = 0.01;
    public static final double VISION_THETA_STDDEV = Units.degreesToRadians(10);

    public static final double MAX_VISION_ANGULAR_VELOCITY = 2 * Math.PI; //Rad/s

    public static final double MIN_TAG_REJECTION_METERS = 5;
    public static final int MIN_TAG_COUNT_REJECTION = 0;

    public static final String[] LIMELIGHTS = {
        "limelight-front",
        "limelight-right"
    };

    public static final double[] POSE_ESTIMATE_WEIGHTS = {
        4.0, // tagCount
        3.0 // avgTagDistance
    };
}
