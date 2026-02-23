package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class RectZone {
    //In meters and from blue alliance corner origin (standard WPILib coordinates)
    public final double minX;
    public final double minY;
    
    public final double maxX;
    public final double maxY;

    public RectZone(double minX, double minY, double maxX, double maxY) {
        this.minX = Math.min(minX, maxX);
        this.minY = Math.min(minY, maxY);

        this.maxX = Math.max(minX, maxX);
        this.maxY = Math.max(minY, maxY);
    }

    //Check if a point is inside the rectangle
    public boolean contains(Translation2d point) {
        return point.getX() >= minX &&
               point.getX() <= maxX &&
               point.getY() >= minY &&
               point.getY() <= maxY;
    }
}