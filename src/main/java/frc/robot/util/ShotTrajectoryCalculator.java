package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class ShotTrajectoryCalculator {
    private static final double GRAVITATIONAL_CONSTANT = 32.2; // Gravitational constant in ft/s^2
    private static final double FLYWHEEL_DIAMETER_INCHES = 4;

    //TODO: Tune values
    private static final double LATENCY_COMPENSATION_SECONDS = 0.040;
    private static final double ANTIDRAG_COEFFICIENT = 1.3; // Coefficient multiplied by horizontal shot velocity to compensate for drag

    //Variables that store the latest calculated values needed to perform a shot
    private static double targetFlywheelRPM = 0;
    private static double targetTurretAngle = 0;
    private static double targetHoodAngle = 0;

    /**
     * @param fuelExitPose The position where the fuel will exit the shooter relative to the field
     * @param robotVelocity The linear velocity of the robot
     * @param targetPose The position of the target we are shooting at
     * @param targetHeight The max height the fuel will ever reach during flight
     */
    public static void update(Pose3d fuelExitPose, ChassisSpeeds robotVelocity, Translation3d targetPose, double targetHeight) {
        double robotXVelocity = Units.metersToFeet(robotVelocity.vxMetersPerSecond);
        double robotZVelocity = Units.metersToFeet(-robotVelocity.vyMetersPerSecond);

        //Adjust the fuel exit pose adjusting for communication latency (assumes constant velocity)
        fuelExitPose.plus(new Transform3d(robotXVelocity * LATENCY_COMPENSATION_SECONDS, 0, robotZVelocity * LATENCY_COMPENSATION_SECONDS, Rotation3d.kZero));

        //t is calculated to be the seconds needed for the ball to reach desired height, and return to goal height, under vacuum conditions
        double t = Math.sqrt(2 * (targetHeight - fuelExitPose.getY()) / GRAVITATIONAL_CONSTANT) + Math.sqrt(2 * (targetHeight - targetPose.getY()) / GRAVITATIONAL_CONSTANT);

        //Calculate launch velocity vector so that it will hit the target in t seconds
        Translation3d fuelVelocity = new Translation3d(
                ANTIDRAG_COEFFICIENT * (targetPose.getX() - fuelExitPose.getX()) / t - robotXVelocity,
                (targetPose.getY() - fuelExitPose.getY()) / t - GRAVITATIONAL_CONSTANT * t / 2,
                ANTIDRAG_COEFFICIENT * (targetPose.getZ() - fuelExitPose.getZ()) / t - robotZVelocity
        );

        targetFlywheelRPM = (fuelVelocity.getNorm() * 720) / (Math.PI * FLYWHEEL_DIAMETER_INCHES);
        targetTurretAngle = Units.radiansToDegrees(Math.atan2(fuelVelocity.getZ(), fuelVelocity.getX()));

        double horizontalSpeed = Math.sqrt(fuelVelocity.getX() * fuelVelocity.getX() + fuelVelocity.getZ() * fuelVelocity.getZ());
        targetHoodAngle = Units.radiansToDegrees(Math.atan2(fuelVelocity.getY(), horizontalSpeed));
    }

    public static double getTargetFlywheelRPM() {
        return targetFlywheelRPM;
    }

    public static double getTurretAngle() {
        return targetTurretAngle; 
    }

    public static double getHoodAngle() { 
        return targetHoodAngle; 
    }
}
