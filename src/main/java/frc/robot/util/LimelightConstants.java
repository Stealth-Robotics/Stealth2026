package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class LimelightConstants {
    //0 = external, 1 = seed internal, 2 = internal, 3 = mt1 + internal, 4 = internal + external
    public static final int DISABLED_IMU_MODE = 1;
    public static final int ENABLED_IMU_MODE = 0;

    public static final double IMU_ALPHA = 0.001;

    public static final int LIMELIGHT_DISABLED_THROTTLE = 0;

    public static final Vector<N3> MT2_STDDEVS = VecBuilder.fill(0.7, 0.7, 99999.0);
    public static final Vector<N3> MT1_STDDEVS = VecBuilder.fill(0.1, 0.1, 0.1);

    public static final double MAX_HEADING_DIVERGENCE_DEGREES = 5;

    public static final double MAX_VELO_METERS_PER_SECOND = 3.0;
    public static final double MAX_ANGULAR_VELO_RADIANS_PER_SECOND = 2.0;

    public static final double MAX_TAG_AMBIGUITY = 0.35; //Safer value = 0.35

    public static final double MAX_SINGLE_TAG_DISTANCE = 3;
    public static final double MAX_MULTI_TAG_DISTANCE = 4;

    public static final int MIN_TAG_COUNT = 0; //1 is safer

    public static final String[] LIMELIGHTS = {
        "limelight-front",
        "limelight-right",
        "limelight-left"
    };
}
