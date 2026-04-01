package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class LimelightConstants {
    public static final int LIMELIGHT_DISABLED_THROTTLE = 100;

    public static final Vector<N3> VISION_STDDEVS = VecBuilder.fill(0.1, 0.1, 99999.0);

    public static final double MAX_VISION_ANGULAR_VELOCITY = 2 * Math.PI; //Rad/s

    public static final double MIN_TAG_REJECTION_METERS = 3.5;
    public static final int MIN_TAG_COUNT_REJECTION = 0; //Could make this value 1 if we have all three cameras

    public static final String[] LIMELIGHTS = {
        "limelight-front",
        "limelight-right"
    };

    public static final double[] POSE_ESTIMATE_WEIGHTS = {
        4.0, // tagCount
        2.0, // avgTagDistance
        3.0  // tagSpan
    };
}
