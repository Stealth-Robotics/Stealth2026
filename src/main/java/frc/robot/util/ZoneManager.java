package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ZoneManager {
    private static Pose2d robotPose = new Pose2d();

    /*  All RectZones should be defined on the Blue Alliance 
     *  and are automatically flipped to the Red Alliance if needed 
     */

    private static final RectZone hub = new RectZone(0, 0, 4, 8.07);

    private static final RectZone leftTrench = new RectZone(4, 6.87, 5.2, 8.07);
    private static final RectZone rightTrench = new RectZone(4, 0, 5.2, 1.25);

    private static final RectZone leftPassing = new RectZone(5.2, 8.07, 16.5, 4);
    private static final RectZone rightPassing = new RectZone(5.2, 0, 16.5, 4);
    
    public static void updateWithRobotPose(Pose2d newRobotPose) {
        robotPose = newRobotPose;
    }

    public static boolean inHubZone() {
        return AllianceUtility.flipRectZone(hub).contains(robotPose.getTranslation());
    }

    public static boolean inPassingZone() {
        return inLeftPassingZone() || inRightPassingZone();
    }

    public static boolean inLeftPassingZone() {
        return AllianceUtility.flipRectZone(leftPassing).contains(robotPose.getTranslation());
    }

    public static boolean inRightPassingZone() {
        return AllianceUtility.flipRectZone(rightPassing).contains(robotPose.getTranslation());
    }

    public static boolean inTrenchZone() {        
        return AllianceUtility.flipRectZone(leftTrench).contains(robotPose.getTranslation()) 
            && AllianceUtility.flipRectZone(rightTrench).contains(robotPose.getTranslation());
    }
}