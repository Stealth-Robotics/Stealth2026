package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class LimelightConstants {
    public static final TagFilterMode TAG_FILTER_MODE = TagFilterMode.ALL_EXCEPT_TRENCH;

    //0 = external, 1 = seed internal, 2 = external, 3 = mt1 + internal, 4 = internal + external
    public static final int DISABLED_IMU_MODE = 1;
    public static final int ENABLED_IMU_MODE = 0;

    public static final double IMU_ALPHA = 0.05; //0.05 old value

    public static final int LIMELIGHT_DISABLED_THROTTLE = 0;

    public static final Vector<N3> STDDEVS = VecBuilder.fill(0.1, 0.1, 0.01);

    public static final double MAX_TAG_AMBIGUITY = 0.35;

    public static final double MAX_SINGLE_TAG_DISTANCE = 3;
    public static final double MAX_MULTI_TAG_DISTANCE = 4;

    public static final int MIN_TAG_COUNT = 0;

    public static final String[] LIMELIGHTS = {
        "limelight-front",
        "limelight-right",
        "limelight-left"
    };

    public enum TagFilterMode {
        ALL(
            new int[] {
                1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
                17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32
            }
        ),
        ALL_EXCEPT_TRENCH(
            new int[] {
                2, 3, 4, 5, 8, 9, 10, 11, 13, 14, 15, 16,
                18, 19, 20, 21, 24, 25, 26, 27, 29, 30, 31, 32
            }
        ),
        HUB_ONLY(
            new int[] {
                2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27
            }
        );

        private final int[] tags;

        TagFilterMode(int[] tags) {
            this.tags = tags;
        }

        public int[] getTags() {
            return tags;
        }
    }
}
