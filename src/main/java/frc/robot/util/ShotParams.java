package frc.robot.util;

import edu.wpi.first.math.geometry.Translation3d;

public record ShotParams(Translation3d target, double maxTrajectoryHeight) {
}
