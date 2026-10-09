package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class SOTMSolver {
    private final double GRAVITY = 9.80665;
    private final double LATENCY = Units.millisecondsToSeconds(30);

    //Taken directly from cad measurements
    private final Transform3d TURRET_OFFSET = new Transform3d(0.14, -0.178, 0.5, Rotation3d.kZero);

    private final InterpolatingDoubleTreeMap heightMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    private final LinearFilter vxFilter = LinearFilter.singlePoleIIR(0.1, Units.millisecondsToSeconds(20));
    private final LinearFilter vyFilter = LinearFilter.singlePoleIIR(0.1, Units.millisecondsToSeconds(20));
    private final LinearFilter vOmegaFilter = LinearFilter.singlePoleIIR(0.1, Units.millisecondsToSeconds(20));

    public SOTMSolver() {
        //Populate interpolation tables with measured values
        heightMap.put(0.0, 2.5);
        heightMap.put(2.0, 2.5);
        heightMap.put(5.14, 3.0);
        heightMap.put(14.0, 6.0);

        rpmMap.put(2.0, 2600.0);
        rpmMap.put(2.18, 2700.0);
        rpmMap.put(3.22, 2900.0);
        rpmMap.put(3.36, 3000.0);
        rpmMap.put(3.64, 3000.0);
        rpmMap.put(4.0, 3100.0);
        rpmMap.put(4.2, 3200.0);
        rpmMap.put(5.14, 3300.0);
        rpmMap.put(8.0, 3800.0);
        rpmMap.put(11.0, 4200.0);
        rpmMap.put(14.0, 6000.0);
    }

    public void resetFilters() {
        vxFilter.reset();
        vyFilter.reset();
        vOmegaFilter.reset();
    }

    /**
     * @param robotPose The pose of the robot on the field
     * @param robotVelocity The field relative velocity of the robot
     * @param targetPose The aiming target
     * @param pass Whether this shot is a pass shot or not
     * @return ShotSolution containing needed setpoints to hit the target
     */
    public ShotSolution solve(Pose3d robotPose, ChassisSpeeds robotVelocity, Translation3d targetPose) {
        double vx = vxFilter.calculate(robotVelocity.vxMetersPerSecond);
        double vy = vyFilter.calculate(robotVelocity.vyMetersPerSecond);
        double vo = vOmegaFilter.calculate(robotVelocity.omegaRadiansPerSecond);
        
        Translation2d turretTangentialVelocityRobotRelative = new Translation2d(
            -vo * TURRET_OFFSET.getY(),
            vo * TURRET_OFFSET.getX()
        );

        Translation2d turretTangentialVelocityFieldRelative = 
            turretTangentialVelocityRobotRelative.rotateBy(robotPose.getRotation().toRotation2d());

        //Calculated turret linear velocity based off of the robot's velocity and the imparted tangential velocity
        double turretVx = vx + turretTangentialVelocityFieldRelative.getX();
        double turretVy = vy + turretTangentialVelocityFieldRelative.getY();

        Pose3d predictedTurretPose = new Pose3d(
            robotPose.getX() + vx * LATENCY,
            robotPose.getY() + vy * LATENCY,
            robotPose.getZ(), //Always zero
            robotPose.getRotation().rotateBy(new Rotation3d(0, 0, -vo * LATENCY))
        ).transformBy(TURRET_OFFSET);

        double distanceToTarget = targetPose.getDistance(predictedTurretPose.getTranslation());

        double shotHeight = Math.max(
            heightMap.get(distanceToTarget),
            Math.max(predictedTurretPose.getZ(), targetPose.getZ()) + 0.1
        );

        double t =
            Math.sqrt(2.0 * (shotHeight - predictedTurretPose.getZ()) / GRAVITY) +
            Math.sqrt(2.0 * (shotHeight - targetPose.getZ()) / GRAVITY);

        double fuelZVelocity = (targetPose.getZ() - predictedTurretPose.getZ()) / t + GRAVITY * t / 2.0;

        Translation3d movingShotVelocity = new Translation3d(
            (targetPose.getX() - predictedTurretPose.getX()) / t - turretVx,
            (targetPose.getY() - predictedTurretPose.getY()) / t - turretVy,
            fuelZVelocity
        );

        Translation3d stationaryShotVelocity = new Translation3d(
            (targetPose.getX() - predictedTurretPose.getX()) / t,
            (targetPose.getY() - predictedTurretPose.getY()) / t,
            fuelZVelocity
        );

        double baseRPM = rpmMap.get(distanceToTarget);

        double stationaryNorm = stationaryShotVelocity.getNorm();
        double veloScale = (stationaryNorm > 1e-4) ? movingShotVelocity.getNorm() / stationaryNorm : 1.0;

        //Scale up the measured RPM by the scale needed to compensate for robot velocity
        double targetFlywheelRPM = baseRPM * veloScale;

        Translation3d tiltAdjustedShotVelocity =
            movingShotVelocity.rotateBy(predictedTurretPose.getRotation().unaryMinus());
        
        //Calculate the necessary turret and hood angles to hit the target
        double targetTurretAngle = -Units.radiansToDegrees(Math.atan2(tiltAdjustedShotVelocity.getY(), tiltAdjustedShotVelocity.getX()));
        double horizontalSpeed = Math.hypot(tiltAdjustedShotVelocity.getX(), tiltAdjustedShotVelocity.getY());
        double targetHoodAngle = 90.0 - Units.radiansToDegrees(Math.atan2(tiltAdjustedShotVelocity.getZ(), horizontalSpeed));

        //Turret velocity feedforward calculation
        double dx = targetPose.getX() - predictedTurretPose.getX();
        double dy = targetPose.getY() - predictedTurretPose.getY();
        double distanceSquared = dx * dx + dy * dy;

        double fieldRelativeTargetTurretVelocity = 0.0;

        if (distanceSquared > 1e-4) //Avoidy the division by zero
            fieldRelativeTargetTurretVelocity = (dx * (-turretVy) - dy * (-turretVx)) / distanceSquared;

        double targetTurretVelocity = Units.radiansToDegrees(fieldRelativeTargetTurretVelocity - vo);

        return new ShotSolution(
            targetFlywheelRPM,
            targetTurretAngle,
            targetTurretVelocity,
            targetHoodAngle,
            distanceToTarget
        );
    }

    public record ShotSolution(
        double rpm,
        double turretAngle,
        double turretVelocity,
        double hoodAngle,
        double metersToTarget
    ) {}
}
