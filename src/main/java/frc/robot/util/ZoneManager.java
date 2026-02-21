package frc.robot.util;

import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ZoneManager {
    private static Pose2d robotPose = new Pose2d();

    private static Rectangle2d BLUE_HUB_ZONE = new Rectangle2d(Translation2d.kZero, Translation2d.kZero);
    private static Rectangle2d RED_HUB_ZONE = new Rectangle2d(Translation2d.kZero, Translation2d.kZero);

    private static Ellipse2d BLUE_HUB_KEEP_OUT_ZONE = new Ellipse2d(Translation2d.kZero, 20);
    private static Ellipse2d RED_HUB_KEEP_OUT_ZONE = new Ellipse2d(Translation2d.kZero, 20);
    
    public static void updateWithRobotPose(Pose2d newRobotPose) {
        robotPose = newRobotPose;
    }

    public static boolean inHubZone() {
        // if (CurrentAlliance.get().equals(Alliance.Blue))
        //     return BLUE_HUB_ZONE.contains(robotPose.getTranslation()) && !BLUE_HUB_KEEP_OUT_ZONE.contains(robotPose.getTranslation());
        // else 
        //     return RED_HUB_ZONE.contains(robotPose.getTranslation()) && !RED_HUB_KEEP_OUT_ZONE.contains(robotPose.getTranslation());

        return false;
    }

    public static boolean inPassingZone() {
        return inLeftPassingZone() || inRightPassingZone();
    }

    public static boolean inLeftPassingZone() {
        Alliance alliance = CurrentAlliance.get();
        return false;
    }

    public static boolean inRightPassingZone() {
        Alliance alliance = CurrentAlliance.get();
        return false;
    }

    public static boolean inTrenchZone() {
        Alliance alliance = CurrentAlliance.get();
        return false;
    }
}