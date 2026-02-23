package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class ShotTrajectoryCalculator {
    private static final double GRAVITATIONAL_CONSTANT = 9.80665; // Gravitational constant in m/s^2
    private static final double FLYWHEEL_DIAMETER_METERS = Units.inchesToMeters(4);

    //TODO: Tune values
    private static final double LATENCY_COMPENSATION_SECONDS = Units.millisecondsToSeconds(50);
    private static final double ANTIDRAG_COEFFICIENT = 1; // Coefficient multiplied by horizontal shot velocity to compensate for drag

    //Variables that store the latest calculated values needed to perform a shot
    private static double targetFlywheelRPM = 0.0;
    private static double targetTurretAngle = 0.0;
    private static double targetHoodAngle = 0.0;

    /**
     * @param fuelExitPose The position where the fuel will exit the shooter relative to the field
     * @param robotVelocity The linear velocity of the robot
     * @param targetPose The position of the target we are shooting at
     * @param targetHeight The max height the fuel will ever reach during flight
     */
    public static void update(Pose3d fuelExitPose, ChassisSpeeds robotVelocity, Translation3d targetPose, double targetHeight) {
        //Scale robot velocity down
        robotVelocity.times(0.9);

        //Adjust the fuel exit pose adjusting for communication latency (assumes constant velocity)
        fuelExitPose = fuelExitPose.plus(
            new Transform3d(
                robotVelocity.vxMetersPerSecond * LATENCY_COMPENSATION_SECONDS,
                robotVelocity.vyMetersPerSecond * LATENCY_COMPENSATION_SECONDS,
                0.0,
                Rotation3d.kZero
            )
        );

        //Cap targetHeight to make sure values don't result in NaN
        targetHeight = Math.max(targetHeight, Math.max(fuelExitPose.getZ(), targetPose.getZ()));

        /*
         *  t is calculated to be the seconds needed for the ball to reach desired height, 
         *  and return to goal height, under vacuum conditions
        */
        double t = 
            Math.sqrt(2.0 * (targetHeight - fuelExitPose.getZ()) / GRAVITATIONAL_CONSTANT) + 
            Math.sqrt(2.0 * (targetHeight - targetPose.getZ()) / GRAVITATIONAL_CONSTANT);

        //Calculate launch velocity vector so that it will hit the target in t seconds
        Translation3d fuelVelocity = new Translation3d(
                ANTIDRAG_COEFFICIENT * (targetPose.getX() - fuelExitPose.getX()) / t - robotVelocity.vxMetersPerSecond,
                ANTIDRAG_COEFFICIENT * (targetPose.getY() - fuelExitPose.getY()) / t - robotVelocity.vyMetersPerSecond,
                (targetPose.getZ() - fuelExitPose.getZ()) / t + GRAVITATIONAL_CONSTANT * t / 2.0
        );

        targetFlywheelRPM = fuelVelocity.getNorm() / (Math.PI * FLYWHEEL_DIAMETER_METERS) * 60.0;

        targetTurretAngle = Units.radiansToDegrees(Math.atan2(fuelVelocity.getY(), fuelVelocity.getX()));

        double horizontalSpeed = Math.sqrt(fuelVelocity.getX() * fuelVelocity.getX() + fuelVelocity.getY() * fuelVelocity.getY());
        targetHoodAngle = 90.0 - Units.radiansToDegrees(Math.atan2(fuelVelocity.getZ(), horizontalSpeed));
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
