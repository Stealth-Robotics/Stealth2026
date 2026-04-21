package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class ZoneManager {
    private static Pose2d robotPose = new Pose2d();

    /* All RectZones should be defined on the Blue Alliance 
     * and are automatically flipped to the Red Alliance if needed 
    */

    private static final RectZone hub = new RectZone(-0.5, -0.5, 4.3, 8.4);

    private static final RectZone passing = new RectZone(5.2, 0, 16.5, 8.07);

    private static final RectZone leftBump = new RectZone(3.6, 4.6, 5.5, 6.5);
    private static final RectZone rightBump = new RectZone(3.6, 1.5, 5.5, 3.5);

    private static final RectZone leftTrench = new RectZone(3.8, 6.87, 5.3, 8.07);
    private static final RectZone rightTrench = new RectZone(3.8, 0, 5.3, 1.25);

    public enum FieldZone {
        HUB,
        PASS,
        TRENCH,
        BUMP
    }
    
    public static void updateRobotPose(Pose2d newRobotPose) {
        robotPose = newRobotPose;
    }

    public static FieldZone getZone() {
        if (inLeftTrenchZone() || inRightTrenchZone())
            return FieldZone.TRENCH;
        else if (inHubZone())
            return FieldZone.HUB;
        else
            return FieldZone.PASS;
    }

    private static boolean inPassingZone() {
        return AllianceUtility.flipRectZone(passing).contains(robotPose.getTranslation());
    }

    public static boolean inBumpZone() {
        return 
            leftBump.contains(robotPose.getTranslation()) ||
            AllianceUtility.forceFlipRectZone(leftBump).contains(robotPose.getTranslation()) ||
            rightBump.contains(robotPose.getTranslation()) ||
            AllianceUtility.forceFlipRectZone(rightBump).contains(robotPose.getTranslation());
    }

    private static boolean inHubZone() {
        return AllianceUtility.flipRectZone(hub).contains(robotPose.getTranslation());
    }

    private static boolean inLeftTrenchZone() {
        return AllianceUtility.forceFlipRectZone(leftTrench).contains(robotPose.getTranslation()) ||
            leftTrench.contains(robotPose.getTranslation());
    }

    private static boolean inRightTrenchZone() {
        return AllianceUtility.forceFlipRectZone(rightTrench).contains(robotPose.getTranslation()) ||
            rightTrench.contains(robotPose.getTranslation());
    }
}