package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class LimelightConstants {
    public static final boolean COMBINE_POSE_ESTIMATES = false;

    public static final int DISABLED_IMU_MODE = 1;
    public static final int AUTO_IMU_MODE = 1;
    public static final int TELEOP_IMU_MODE = 3; //Or 4

    public static final int[] ALLOWED_TAGS = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
        17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32
    };

    public static final int LIMELIGHT_DISABLED_THROTTLE = 200;

    public static final double VISION_XY_STDDEV = 0.05;
    public static final double VISION_THETA_STDDEV = Units.degreesToRadians(15);

    public static final Vector<N3> STDDEVS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(15));

    public static final double MAX_VISION_ANGULAR_VELOCITY = 2.0; //Rad/s

    public static final double MAX_TAG_DISTANCE = 3.5;
    public static final int MIN_TAG_COUNT_REJECTION = 1;
    public static final double MAX_TAG_AMBIGUITY = 0.5;

    public static final String[] LIMELIGHTS = {
        "limelight-front",
        "limelight-right",
        "limelight-left"
    };
}
