package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class LimelightConstants {
    public static final int LIMELIGHT_DISABLED_THROTTLE = 100;

    public static final Vector<N3> VISION_STDDEVS = VecBuilder.fill(0.01, 0.01, 99999.0);

    public static final double MAX_VISION_ANGULAR_VELOCITY = 2 * Math.PI; //Rad/s

    public static final double MIN_TAG_REJECTION_METERS = 4.2;
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
