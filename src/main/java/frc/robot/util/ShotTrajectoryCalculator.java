package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class ShotTrajectoryCalculator {
    private static final double GRAVITATIONAL_CONSTANT = 9.80665; // Gravitational constant in m/s^2

    //TODO: Tune values
    private static final double LATENCY_COMPENSATION_SECONDS = Units.millisecondsToSeconds(50);

    private static final InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap() {{
        put(2.18, 2700.0);
        put(2.89, 3000.0);
        put(4.299, 3400.0);
        put(5.3, 3800.0);
    }};

    //Variables that store the latest calculated values needed to perform a shot
    private static double targetFlywheelRPM = 0.0;
    private static double targetTurretAngle = 0.0;
    private static double targetHoodAngle = 0.0;

    /**
     * @param fuelExitPose The position where the fuel will exit the shooter relative to the field
     * @param robotVelocity The linear velocity of the robot
     * @param targetPose The position of the target we are shooting at
     * @param timeOfFlight The projectile time of flight under vacuum conditions
     */
    public static void updateTimed(Pose3d fuelExitPose, ChassisSpeeds robotVelocity, Translation3d targetPose, double timeOfFlight) {
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

        Translation3d movingShotVelocity = new Translation3d(
                (targetPose.getX() - fuelExitPose.getX()) / timeOfFlight + robotVelocity.vyMetersPerSecond,
                (targetPose.getY() - fuelExitPose.getY()) / timeOfFlight - robotVelocity.vxMetersPerSecond,
                (targetPose.getZ() - fuelExitPose.getZ()) / timeOfFlight + GRAVITATIONAL_CONSTANT * timeOfFlight / 2.0
        );

        Translation3d stationaryShotVelocity = new Translation3d(
            (targetPose.getX() - fuelExitPose.getX()) / timeOfFlight,
            (targetPose.getY() - fuelExitPose.getY()) / timeOfFlight,
            (targetPose.getZ() - fuelExitPose.getZ()) / timeOfFlight + GRAVITATIONAL_CONSTANT * timeOfFlight / 2.0
        );

        double metersToGoal = targetPose.getDistance(fuelExitPose.getTranslation());

        double baseRPM = distanceToRPM.get(metersToGoal);
        double veloScale = movingShotVelocity.getNorm() / stationaryShotVelocity.getNorm();

        //Scale up the measured RPM by the scale needed to compensate for robot velocity
        targetFlywheelRPM = baseRPM * veloScale;

        targetTurretAngle = Units.radiansToDegrees(Math.atan2(movingShotVelocity.getY(), movingShotVelocity.getX()));

        double horizontalSpeed = Math.sqrt(Math.pow(movingShotVelocity.getX(), 2) + Math.pow(movingShotVelocity.getY(), 2));
        targetHoodAngle = 90.0 - Units.radiansToDegrees(Math.atan2(movingShotVelocity.getZ(), horizontalSpeed));
    } 

    /**
     * @param fuelExitPose The position where the fuel will exit the shooter relative to the field
     * @param robotVelocity The linear velocity of the robot
     * @param targetPose The position of the target we are shooting at
     * @param targetHeight The max height the fuel will ever reach during flight
     */
    public static void update(Pose3d fuelExitPose, ChassisSpeeds robotVelocity, Translation3d targetPose, double targetHeight) {
        double t = 
            Math.sqrt(2.0 * (targetHeight - fuelExitPose.getZ()) / GRAVITATIONAL_CONSTANT) + 
            Math.sqrt(2.0 * (targetHeight - targetPose.getZ()) / GRAVITATIONAL_CONSTANT);
        updateTimed(fuelExitPose, robotVelocity, targetPose, t);
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
