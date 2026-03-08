package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ZoneManager {
    private static Pose2d robotPose = new Pose2d();
    private static ChassisSpeeds robotFieldSpeeds = new ChassisSpeeds();

    /* All RectZones should be defined on the Blue Alliance 
     * and are automatically flipped to the Red Alliance if needed 
    */

    private static final RectZone hub = new RectZone(-0.5, -0.5, 4, 8.07);

    private static final RectZone passing = new RectZone(5.2, 0, 16.5, 8.07);

    private static final RectZone leftTrench = new RectZone(4, 6.87, 5.2, 8.07);
    private static final RectZone rightTrench = new RectZone(4, 0, 5.2, 1.25);

    private static final double ROBOT_FUTURE_POSE_PREDICTION_SECONDS = 0.5;

    public enum FieldZone {
        HUB,
        PASS,
        TRENCH,
        UNKNOWN
    }
    
    public static void updateRobotPositionAndVelocity(Pose2d newRobotPose, ChassisSpeeds newRobotSpeeds) {
        robotPose = newRobotPose;
        robotFieldSpeeds = newRobotSpeeds;
    }

    public static FieldZone getZone() {
        if (inHubZone())
            return FieldZone.HUB;
        else if (inPassingZone())
            return FieldZone.PASS;
        else if
                (inLeftTrenchZone() || inRightTrenchZone() ||
                inLeftTrenchZoneFuture() || inRightTrenchZoneFuture())
            return FieldZone.TRENCH;
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

    /*
     * Checks if we are heading towards the trench zone in the next fraction of a second
     */
    private static boolean inLeftTrenchZoneFuture() {
        return AllianceUtility.flipRectZone(leftTrench).contains(getFutureRobotPose());
    }

    /*
     * Checks if we are heading towards the trench zone in the next fraction of a second
     */
    private static boolean inRightTrenchZoneFuture() {
        return AllianceUtility.flipRectZone(rightTrench).contains(getFutureRobotPose());
    }

    private static Translation2d getFutureRobotPose() {
        return robotPose.getTranslation().plus(
            new Translation2d(
                robotFieldSpeeds.vxMetersPerSecond * ROBOT_FUTURE_POSE_PREDICTION_SECONDS,
                robotFieldSpeeds.vyMetersPerSecond * ROBOT_FUTURE_POSE_PREDICTION_SECONDS
            )
        );
    }
}