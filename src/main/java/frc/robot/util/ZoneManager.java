package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class ZoneManager {
    private static Pose2d robotPose = new Pose2d();

    /* All RectZones should be defined on the Blue Alliance 
     * and are automatically flipped to the Red Alliance if needed 
    */

    private static final RectZone hub = new RectZone(-0.5, -0.5, 4, 8.07);

    private static final RectZone passing = new RectZone(5.3, 0, 16.5, 8.07);

    private static final RectZone leftTrench = new RectZone(3.8, 6.87, 5.3, 8.07);
    private static final RectZone rightTrench = new RectZone(3.8, 0, 5.3, 1.25);

    public enum FieldZone {
        HUB,
        PASS,
        TRENCH,
        UNKNOWN
    }
    
    public static void updateRobotPositionAndVelocity(Pose2d newRobotPose) {
        robotPose = newRobotPose;
    }

    public static FieldZone getZone() {
        //Make sure trench zone overrides all others for safety reasons
        if (inLeftTrenchZone() || inRightTrenchZone())
            return FieldZone.TRENCH;
        else if (inHubZone())
            return FieldZone.HUB;
        else if (inPassingZone())
            return FieldZone.PASS;
        else
            return FieldZone.UNKNOWN;
    }

    private static boolean inHubZone() {
        return AllianceUtility.flipRectZone(hub).contains(robotPose.getTranslation());
    }

    private static boolean inPassingZone() {
        return AllianceUtility.flipRectZone(passing).contains(robotPose.getTranslation());
    }

    private static boolean inLeftTrenchZone() {
        return AllianceUtility.flipRectZone(leftTrench).contains(robotPose.getTranslation());
    }

    private static boolean inRightTrenchZone() {
        return AllianceUtility.flipRectZone(rightTrench).contains(robotPose.getTranslation());
    }
}