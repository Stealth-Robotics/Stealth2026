package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceUtility {
    public static Alliance latestAlliance = Alliance.Blue;

    public static final double FIELD_LENGTH_METERS = 16.540988;
    public static final double FIELD_WIDTH_METERS = 8.069326;
    
    public static final Pose2d FIELD_CENTER_POINT = new Pose2d(FIELD_LENGTH_METERS / 2.0, FIELD_WIDTH_METERS / 2.0, Rotation2d.kZero);

    public static Alliance getAlliance() {
        return latestAlliance;
    }

    public static boolean isWithinField(Pose2d p) {
        return p.getX() >= 0 && p.getY() >= 0 && p.getX() <= FIELD_LENGTH_METERS && p.getY() <= FIELD_WIDTH_METERS;
    }

    /**
     * Returns the original Pose2d but rotated 180 degrees around the field's center point (transitioning a coordinate
     * from the BLUE alliance to the RED and vice versa) if the latest alliance is RED, otherwise it returns the original
     * Pose2d.
     */
    public static Pose2d flipPose(Pose2d original) {
        if (latestAlliance.equals(Alliance.Red)) {
            return new Pose2d(
                FIELD_CENTER_POINT.getX() + (FIELD_CENTER_POINT.getX() - original.getX()), 
                FIELD_CENTER_POINT.getY() + (FIELD_CENTER_POINT.getY() - original.getY()), 
                original.getRotation().rotateBy(Rotation2d.k180deg)
            );
        }
        return original;
    }

    public static Translation3d flipPose(Translation3d original) {
        if (latestAlliance.equals(Alliance.Red)) {
            return new Translation3d(
                FIELD_CENTER_POINT.getX() + (FIELD_CENTER_POINT.getX() - original.getX()),
                FIELD_CENTER_POINT.getY() + (FIELD_CENTER_POINT.getY() - original.getY()), 
                original.getZ()
            );
        }
        return original;
    }

    public static double flipYCoordinate(double originalY) {
        if (latestAlliance.equals(Alliance.Red)) {
            return FIELD_CENTER_POINT.getY() + (FIELD_CENTER_POINT.getY() - originalY);
        }
        return originalY;
    }

    public static ShotParams flipPose(ShotParams original) {
        if (latestAlliance.equals(Alliance.Red)) {
            return new ShotParams(flipPose(original.target()), original.maxTrajectoryHeight());
        }
        return original;
    }

    public static RectZone flipRectZone(RectZone original) {
        if (latestAlliance.equals(Alliance.Red)) {
            return new RectZone(
                FIELD_CENTER_POINT.getX() + (FIELD_CENTER_POINT.getX() - original.minX),
                FIELD_CENTER_POINT.getY() + (FIELD_CENTER_POINT.getY() - original.minY), 
                FIELD_CENTER_POINT.getX() + (FIELD_CENTER_POINT.getX() - original.maxX),
                FIELD_CENTER_POINT.getY() + (FIELD_CENTER_POINT.getY() - original.maxY)
            );
        }
        return original;
    }

    public static RectZone forceFlipRectZone(RectZone original) {
        return new RectZone(
            FIELD_CENTER_POINT.getX() + (FIELD_CENTER_POINT.getX() - original.minX),
            FIELD_CENTER_POINT.getY() + (FIELD_CENTER_POINT.getY() - original.minY), 
            FIELD_CENTER_POINT.getX() + (FIELD_CENTER_POINT.getX() - original.maxX),
            FIELD_CENTER_POINT.getY() + (FIELD_CENTER_POINT.getY() - original.maxY)
        );
    }

    public static void update() {
        latestAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    }
}
