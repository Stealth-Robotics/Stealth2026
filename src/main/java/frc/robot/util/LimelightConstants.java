package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class LimelightConstants {
    public static final int LIMELIGHT_DISABLED_THROTTLE = 200;

    public static final double VISION_XY_STDDEV = 0.05;
    public static final double VISION_THETA_STDDEV = Units.degreesToRadians(15);

    public static final double MAX_VISION_ANGULAR_VELOCITY = Math.PI; //Rad/s

    public static final double MIN_TAG_REJECTION_METERS = 3.5;
    public static final int MIN_TAG_COUNT_REJECTION = 0;

    public static final String[] LIMELIGHTS = {
        "limelight-front",
        "limelight-right"
        // "limelight-left"
    };
}
