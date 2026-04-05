package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class LimelightConstants {
    public static final boolean COMBINE_POSE_ESTIMATES = false;

    public static final int LIMELIGHT_DISABLED_THROTTLE = 200;

    public static final double VISION_XY_STDDEV = 0.05;
    public static final double VISION_THETA_STDDEV = Units.degreesToRadians(15);

    public static final Vector<N3> STDDEVS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(15));

    public static final double MAX_VISION_ANGULAR_VELOCITY = Math.PI; //Rad/s

    public static final double MAX_TAG_DISTANCE = 4.5;
    public static final int MIN_TAG_COUNT_REJECTION = 0;

    public static final String[] LIMELIGHTS = {
        "limelight-front",
        "limelight-right"
        // "limelight-left"
    };
}
