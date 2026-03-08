package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class ShotCalculator {
    private static final double GRAVITATIONAL_CONSTANT = 9.80665; // Gravitational constant in m/s^2

    private static final double systemLatency = Units.millisecondsToSeconds(20);
    private static double visionLatency = 0;

    //Time needed for ball to travel through feeder towards the flywheel
    private static final double mechanismLatency = Units.millisecondsToSeconds(0);

    private static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap() {{
        // put(2.03, 2700.0);
        // put(2.5, 2700.0);
        // put(3.0, 2700.0);
        // put(3.25, 2900.0);
        // put(3.59, 3000.0);
        // put(5.25, 3700.0);
        put(2.18, 2700.0);
        put(2.89, 3000.0);
        put(4.299, 3400.0);
        put(5.3, 3800.0);
    }};

    //Variables that store the latest calculated values needed to perform a shot
    private static double targetFlywheelRPM = 0.0;
    private static double targetTurretAngle = 0.0;
    private static double targetHoodAngle = 0.0;

    private static double previousRobotVx = 0.0;
    private static double previousRobotVy = 0.0;

    public static void updateVisionLatency(double ms) {
        visionLatency = Units.millisecondsToSeconds(ms);
    }

    /**
     * @param fuelExitPose The position where the fuel will exit the shooter relative to the field
     * @param robotVelocity The linear velocity of the robot (field relative)
     * @param targetPose The position of the target we are shooting at
     * @param targetHeight The max height the fuel will ever reach during flight
     */
    public static void update(Pose3d fuelExitPose, ChassisSpeeds robotVelocity, Translation3d targetPose, double targetHeight) {
        double totalLatencySeconds = visionLatency + systemLatency + mechanismLatency;
        
        //Estimate the robot's velocity assuming a constant 20 ms periodic loop
        Translation2d robotAcceleration = new Translation2d(
            (robotVelocity.vxMetersPerSecond - previousRobotVx) / Units.millisecondsToSeconds(20),
            (robotVelocity.vyMetersPerSecond - previousRobotVy) / Units.millisecondsToSeconds(20)
        );

        //Adjust the fuel exit pose adjusting for communication latency (assumes constant velocity)
        fuelExitPose = fuelExitPose.plus(
            new Transform3d(
                robotVelocity.vxMetersPerSecond * totalLatencySeconds 
                    + (0.5 * robotAcceleration.getX() * Math.pow(totalLatencySeconds, 2)),
                robotVelocity.vyMetersPerSecond * totalLatencySeconds 
                    + (0.5 * robotAcceleration.getY() * Math.pow(totalLatencySeconds, 2)),
                0.0,
                Rotation3d.kZero
            )
        );

        //Clamp targetHeight to make sure values don't result in a NaN result
        targetHeight = Math.max(targetHeight, Math.max(fuelExitPose.getZ(), targetPose.getZ()));

        /*
         *  t is calculated to be the seconds needed for the ball to reach desired height, 
         *  and return to goal height, under vacuum conditions
        */
        double t = 
            Math.sqrt(2.0 * (targetHeight - fuelExitPose.getZ()) / GRAVITATIONAL_CONSTANT) + 
            Math.sqrt(2.0 * (targetHeight - targetPose.getZ()) / GRAVITATIONAL_CONSTANT);

        double fuelZVelo = (targetPose.getZ() - fuelExitPose.getZ()) / t + GRAVITATIONAL_CONSTANT * t / 2.0;

        Translation3d movingShotVelocity = new Translation3d(
            (targetPose.getX() - fuelExitPose.getX()) / t - robotVelocity.vxMetersPerSecond,
            (targetPose.getY() - fuelExitPose.getY()) / t - robotVelocity.vyMetersPerSecond,
            fuelZVelo
        );

        Translation3d stationaryShotVelocity = new Translation3d(
            (targetPose.getX() - fuelExitPose.getX()) / t,
            (targetPose.getY() - fuelExitPose.getY()) / t,
            fuelZVelo
        );

        double metersToGoal = targetPose.getDistance(fuelExitPose.getTranslation());

        double baseRPM = distanceToRPM.get(metersToGoal); 
        double veloScale = movingShotVelocity.getNorm() / stationaryShotVelocity.getNorm();

        //Scale up the measured RPM by the scale needed to compensate for robot velocity
        targetFlywheelRPM = baseRPM * veloScale;

        targetTurretAngle = Units.radiansToDegrees(Math.atan2(movingShotVelocity.getY(), movingShotVelocity.getX()));

        double horizontalSpeed = Math.sqrt(Math.pow(movingShotVelocity.getX(), 2) + Math.pow(movingShotVelocity.getY(), 2));
        targetHoodAngle = 90.0 - Units.radiansToDegrees(Math.atan2(movingShotVelocity.getZ(), horizontalSpeed));

        //Store velocities for acceleration calculations each loop
        previousRobotVx = robotVelocity.vxMetersPerSecond;
        previousRobotVy = robotVelocity.vyMetersPerSecond;
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
