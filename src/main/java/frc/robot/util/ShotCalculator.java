package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class ShotCalculator {
    private static final double GRAVITATIONAL_CONSTANT = 9.80665; // Gravitational constant in m/s^2

    private static final double systemPeriod = Units.millisecondsToSeconds(20);

    //Time needed for ball to travel through feeder towards the flywheel
    private static final double mechanismLatency = Units.millisecondsToSeconds(0);

    private static final InterpolatingDoubleTreeMap hubDistanceToRPM = new InterpolatingDoubleTreeMap() {{
        put(1.71, 2450.0);
        put(2.04, 2700.0);
        put(2.5, 2950.0);
        put(2.75, 3050.0);
        put(3.0, 3150.0);
        put(3.28, 3200.0);
        put(3.5, 3215.0);
        put(4.0, 3250.0);
        put(4.5, 3302.0);
        put(5.0, 3580.0);
    }};

    private static final InterpolatingDoubleTreeMap passingDistanceToRPM = new InterpolatingDoubleTreeMap() {{
        put(3.0, 3200.0);
        put(5.0, 3600.0);
        put(8.0, 3700.0);
    }};

    //Velocity smoothing filters
    private static final LinearFilter vxFilter = LinearFilter.singlePoleIIR(0.1, systemPeriod);
    private static final LinearFilter vyFilter = LinearFilter.singlePoleIIR(0.1, systemPeriod);
    private static final LinearFilter vOmegaFilter = LinearFilter.singlePoleIIR(0.1, systemPeriod);

    //Variables that store the latest calculated values needed to perform a shot
    private static double targetFlywheelRPM = 0.0;
    private static double targetTurretAngle = 0.0;
    private static double targetHoodAngle = 0.0;

    private static double previousRobotVx = 0.0;
    private static double previousRobotVy = 0.0;
    private static double previousRobotVOmega = 0.0;

    public static void resetFilters() {
        vxFilter.reset();
        vyFilter.reset();
    }

    /**
     * @param fuelExitPose The position where the fuel will exit the shooter relative to the field
     * @param robotVelocity The linear velocity of the robot (robot relative)
     * @param targetPose The position of the target we are shooting at
     * @param targetHeight The max height the fuel will ever reach during flight
     */
    public static void update(Pose3d fuelExitPose, ChassisSpeeds robotVelocity, Translation3d targetPose, double targetHeight, boolean isPassShot) {
        double totalLatencySeconds = systemPeriod + mechanismLatency;

        double filteredVx = vxFilter.calculate(robotVelocity.vxMetersPerSecond);
        double filteredVy = vyFilter.calculate(robotVelocity.vyMetersPerSecond);
        double filteredVOmega = vOmegaFilter.calculate(robotVelocity.omegaRadiansPerSecond);
        
        //Estimate the robot's velocity assuming a constant 20 ms periodic loop
        Translation3d robotAcceleration = new Translation3d(
            (filteredVx - previousRobotVx) / systemPeriod,
            (filteredVy - previousRobotVy) / systemPeriod,
            (filteredVOmega - previousRobotVOmega) / systemPeriod
        );

        //Adjust the fuel exit pose adjusting for communication latency (assumes constant velocity)
        fuelExitPose = fuelExitPose.plus(
            new Transform3d(
                filteredVx * totalLatencySeconds 
                    + (0.5 * robotAcceleration.getX() * Math.pow(totalLatencySeconds, 2)),
                filteredVy * totalLatencySeconds 
                    + (0.5 * robotAcceleration.getY() * Math.pow(totalLatencySeconds, 2)),
                0.0,
                new Rotation3d(
                    0, 0,
                    filteredVOmega * totalLatencySeconds
                        + (0.5 * robotAcceleration.getZ() * Math.pow(totalLatencySeconds, 2))
                )
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

        double baseRPM = (isPassShot) ? passingDistanceToRPM.get(metersToGoal) : hubDistanceToRPM.get(metersToGoal);
        double veloScale = movingShotVelocity.getNorm() / stationaryShotVelocity.getNorm();

        //Scale up the measured RPM by the scale needed to compensate for robot velocity
        targetFlywheelRPM = baseRPM * veloScale;

        targetTurretAngle = Units.radiansToDegrees(Math.atan2(movingShotVelocity.getY(), movingShotVelocity.getX()));

        double horizontalSpeed = Math.sqrt(Math.pow(movingShotVelocity.getX(), 2) + Math.pow(movingShotVelocity.getY(), 2));
        targetHoodAngle = 90.0 - Units.radiansToDegrees(Math.atan2(movingShotVelocity.getZ(), horizontalSpeed));

        //Store velocities for acceleration calculations each loop
        previousRobotVx = filteredVx;
        previousRobotVy = filteredVy;
        previousRobotVOmega = filteredVOmega;
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
