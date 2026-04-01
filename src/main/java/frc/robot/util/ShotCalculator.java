package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ShotCalculator.ShooterState;

public class ShotCalculator {
    private static final double GRAVITATIONAL_CONSTANT = 9.80665; // Gravitational constant in m/s^2

    private static final double systemPeriod = Units.millisecondsToSeconds(20);

    //Time needed for ball to travel through feeder towards the flywheel
    private static final double mechanismLatency = Units.millisecondsToSeconds(15);

private static final InterpolatingTreeMap<Double, ShooterState> hubDistanceToState = 
    new InterpolatingTreeMap<Double, ShooterState>(InverseInterpolator.forDouble(), ShooterState::interpolate) {{
        put(1.96, new ShooterState(2600.0, 8.0));
        put(2.35, new ShooterState(2800.0, 10.0));
        put(2.5,  new ShooterState(2800.0, 11.5));
        put(2.75, new ShooterState(2900.0, 12.0));
        put(3.0,  new ShooterState(2925.0, 12.0));
        put(3.5,  new ShooterState(3000.0, 12.0));
        put(4.0,  new ShooterState(3100.0, 12.0));
        put(4.9,  new ShooterState(3200.0, 12.0));
    }};

    private static final InterpolatingTreeMap<Double, ShooterState> passingDistanceToRPM = new InterpolatingTreeMap<Double, ShooterState>(InverseInterpolator.forDouble(), ShooterState::interpolate) {{
        put(3.0, new ShooterState(3000.0, 12.0));
        put(5.0, new ShooterState(3200.0, 12.0));
        put(8.0, new ShooterState(3800.0, 12.0));
        put(11.0, new ShooterState(4200.0, 12.0));
        put(14.0, new ShooterState(6000.0, 12.0));
    }};

    //Velocity smoothing filters
    private static final LinearFilter vxFilter = LinearFilter.singlePoleIIR(0.1, systemPeriod);
    private static final LinearFilter vyFilter = LinearFilter.singlePoleIIR(0.1, systemPeriod);
    private static final LinearFilter vOmegaFilter = LinearFilter.singlePoleIIR(0.1, systemPeriod);

    //Variables that store the latest calculated values needed to perform a shot
    private static double targetFlywheelRPM = 0.0;
    private static double targetTurretAngle = 0.0;
    private static double targetHoodAngle = 0.0;
    private static double targetFeederVolts = 0.0;

    record ShooterState(double shooterRpm, double feederVolts) implements Interpolatable<ShooterState> {
        @Override
        public ShooterState interpolate(ShooterState endValue, double t) {
            return new ShooterState(
                shooterRpm + (endValue.shooterRpm - shooterRpm) * t,
                feederVolts + (endValue.feederVolts - feederVolts) * t
            );
        }
    }
    
    public static void resetFilters() {
        vxFilter.reset();
        vyFilter.reset();
        vOmegaFilter.reset();
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

        //Adjust the fuel exit pose adjusting for communication latency (assumes constant velocity)
        fuelExitPose = fuelExitPose.plus(
            new Transform3d(
                filteredVx * totalLatencySeconds,
                filteredVy * totalLatencySeconds,
                0,
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
            (targetPose.getX() - fuelExitPose.getX()) / t - filteredVx,
            (targetPose.getY() - fuelExitPose.getY()) / t - filteredVy,
            fuelZVelo
        );

        Translation3d stationaryShotVelocity = new Translation3d(
            (targetPose.getX() - fuelExitPose.getX()) / t,
            (targetPose.getY() - fuelExitPose.getY()) / t,
            fuelZVelo
        );

        double metersToGoal = targetPose.getDistance(fuelExitPose.getTranslation());
        
        ShooterState state = (isPassShot) ? passingDistanceToRPM.get(metersToGoal) : hubDistanceToState.get(metersToGoal);
        double veloScale = movingShotVelocity.getNorm() / stationaryShotVelocity.getNorm();

        //Scale up the measured RPM by the scale needed to compensate for robot velocity
        targetFlywheelRPM = state.shooterRpm * veloScale;
        targetFeederVolts = state.feederVolts;

        targetTurretAngle = Units.radiansToDegrees(
            Math.atan2(movingShotVelocity.getY(), movingShotVelocity.getX()) - (filteredVOmega * totalLatencySeconds)
        );

        double horizontalSpeed = Math.sqrt(Math.pow(movingShotVelocity.getX(), 2) + Math.pow(movingShotVelocity.getY(), 2));
        targetHoodAngle = 90.0 - Units.radiansToDegrees(Math.atan2(movingShotVelocity.getZ(), horizontalSpeed));
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

    public static double getFeederVolts() {
        return targetFeederVolts;
    }
}
