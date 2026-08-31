package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class ShotCalculator {
    private static final double GRAVITATIONAL_CONSTANT = 9.80665; // Gravitational constant in m/s^2

    private static final double systemPeriod = Units.millisecondsToSeconds(20);
    private static final double communicationLatency = Units.millisecondsToSeconds(10);

    private static final InterpolatingDoubleTreeMap hubDistanceToRPM = new InterpolatingDoubleTreeMap() {{
        put(5.14, 3300.0);
        put(4.2, 3200.0);
        put(4.0, 3100.0);
        put(3.64, 3000.0);
        put(3.36, 3000.0);
        put(3.22, 2900.0);
        put(2.18, 2700.0);
        put(2.0, 2600.0);
    }};

    private static final InterpolatingDoubleTreeMap passingDistanceToRPM = new InterpolatingDoubleTreeMap() {{
        put(3.0, 2800.0);
        put(5.0, 3100.0);
        put(8.0, 3800.0);
        put(11.0, 4200.0);
        put(14.0, 6000.0);
    }};

    //Velocity smoothing filters
    private static final LinearFilter vxFilter = LinearFilter.singlePoleIIR(0.1, systemPeriod);
    private static final LinearFilter vyFilter = LinearFilter.singlePoleIIR(0.1, systemPeriod);
    private static final LinearFilter vOmegaFilter = LinearFilter.singlePoleIIR(0.1, systemPeriod);

    //Structure for holding the calculations returned by this class
    public record SOTMResult(
        double rpm, 
        double turretAngle, 
        double turretVelocity, 
        double hoodAngle, 
        double distance
    ) {}

    public static void resetFilters() {
        vxFilter.reset();
        vyFilter.reset();
        vOmegaFilter.reset();
    }

    /**
     * @param fuelExitPose The position where the fuel will exit the shooter relative to the field
     * @param fieldRelativeRobotVelocity The linear velocity of the robot
     * @param targetPose The position of the target we are shooting at
     * @param targetHeight The max height the fuel will ever reach during flight
     */
    public static SOTMResult calculate(Pose3d fuelExitPose, ChassisSpeeds fieldRelativeRobotVelocity, Translation3d targetPose, double targetHeight, boolean isPassShot) {
        double totalLatencySeconds = systemPeriod + communicationLatency;
        
        //Filtered field relative robot velocity components
        double vx = vxFilter.calculate(fieldRelativeRobotVelocity.vxMetersPerSecond);
        double vy = vyFilter.calculate(fieldRelativeRobotVelocity.vyMetersPerSecond);
        double vo = vOmegaFilter.calculate(fieldRelativeRobotVelocity.omegaRadiansPerSecond);

        //Adjust the fuel exit pose adjusting for communication latency (assumes constant velocity)
        fuelExitPose = fuelExitPose.plus(new Transform3d(
            vx * totalLatencySeconds,
            vy * totalLatencySeconds,
            0,
            Rotation3d.kZero
        ));

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
        
        //Target coordinates relative to turret/shooter in field coordinates
        double dx = targetPose.getX() - fuelExitPose.getX();
        double dy = targetPose.getY() - fuelExitPose.getY();

        Translation3d movingShotVelocity = new Translation3d(dx / t - vx, dy / t - vy, fuelZVelo);
        Translation3d stationaryShotVelocity = new Translation3d(dx / t, dy / t, fuelZVelo);

        double metersToGoal = targetPose.getDistance(fuelExitPose.getTranslation());

        double baseRPM = (isPassShot) ? passingDistanceToRPM.get(metersToGoal) : hubDistanceToRPM.get(metersToGoal);
        double veloScale = movingShotVelocity.getNorm() / stationaryShotVelocity.getNorm();

        //Scale up the measured RPM by the scale needed to compensate for robot velocity
        double targetFlywheelRPM = baseRPM * veloScale;

        double targetTurretAngle = Units.radiansToDegrees(
            Math.atan2(movingShotVelocity.getY(), 
            movingShotVelocity.getX()) - (vo * totalLatencySeconds)
        );
        
        double horizontalSpeed = Math.hypot(movingShotVelocity.getX(), movingShotVelocity.getY());
        double targetHoodAngle = 90 - Units.radiansToDegrees(Math.atan2(movingShotVelocity.getZ(), horizontalSpeed));
        
        //How fast the line of sight between the target and turret is changing
        double lineOfSightVelocity = (dx * vy - dy * vx) / (dx * dx + dy * dy);

        //Calculate the necessary turret velocity to match the robot's velocity to aim smoothly and not lag behind
        double targetTurretVelocity = lineOfSightVelocity - vo;

        return new SOTMResult(targetFlywheelRPM, targetTurretAngle, targetTurretVelocity, targetHoodAngle, metersToGoal);
    }
}
