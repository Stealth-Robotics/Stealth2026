package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class ZoneManager {
    private static Pose2d robotPose = new Pose2d();

    /* All RectZones should be defined on the Blue Alliance 
     * and are automatically flipped to the Red Alliance if needed 
    */

    private static final RectZone hub = new RectZone(-0.5, -0.5, 4, 8.07);

    private static final RectZone leftPassing = new RectZone(5.4, 8.07, 16.5, 4);
    private static final RectZone rightPassing = new RectZone(5.4, 0, 16.5, 4);

    private static final RectZone leftTrench = new RectZone(3.8, 6.87, 5.4, 8.07);
    private static final RectZone rightTrench = new RectZone(3.8, 0, 5.4, 1.25);

    public enum FieldZone {
        HUB,
        LEFT_PASS,
        RIGHT_PASS,
        TRENCH,
        UNKNOWN
    }
    
    public static void updateWithRobotPose(Pose2d newRobotPose) {
        robotPose = newRobotPose;
    }

    public static FieldZone getZone() {
        if (inHubZone())
            return FieldZone.HUB;
        else if (inLeftPassingZone())
            return FieldZone.LEFT_PASS;
        else if (inRightPassingZone())
            return FieldZone.RIGHT_PASS;
        else if (inLeftTrenchZone() || inRightTrenchZone())
            return FieldZone.TRENCH;
        else
            return FieldZone.UNKNOWN;
    }

    private static boolean inHubZone() {
        return AllianceUtility.flipRectZone(hub).contains(robotPose.getTranslation());
    }

    private static boolean inLeftPassingZone() {
        return AllianceUtility.flipRectZone(leftPassing).contains(robotPose.getTranslation());
    }

    private static boolean inRightPassingZone() {
        return AllianceUtility.flipRectZone(rightPassing).contains(robotPose.getTranslation());
    }

    private static boolean inLeftTrenchZone() {
        return AllianceUtility.flipRectZone(leftTrench).contains(robotPose.getTranslation());
    }

    private static boolean inRightTrenchZone() {
        return AllianceUtility.flipRectZone(rightTrench).contains(robotPose.getTranslation());
    }
}