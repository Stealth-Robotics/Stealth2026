package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class FieldConstants {
    

    // Climb Positions

    public static Pose2d BLUE_LEFT_TOWER_CLIMB_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static Pose2d BLUE_RIGHT_TOWER_CLIMB_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    public static Pose2d RED_LEFT_TOWER_CLIMB_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static Pose2d RED_RIGHT_TOWER_CLIMB_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    public static Pose2d BLUE_LEFT_AUTO_CLIMB_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static Pose2d BLUE_RIGHT_AUTO_CLIMB_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    public static Pose2d RED_LEFT_AUTO_CLIMB_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static Pose2d RED_RIGHT_AUTO_CLIMB_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    // Passing Positions

    public static Pose2d RED_LEFT_PASSING_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static Pose2d RED_RIGHT_PASSING_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    public static Pose2d BLUE_LEFT_PASSING_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static Pose2d BLUE_RIGHT_PASSING_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
}
