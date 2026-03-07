package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotTrajectoryCalculator {
    private static ShotTrajectoryCalculator instance;

    // Offset from robot center to turret center, in robot-relative frame (meters).
    // x = 0.19m forward, y = -0.2m right.
    public static final Translation2d ROBOT_TO_TURRET = new Translation2d(0.19, -0.2);

    // Accounts for processing/CAN latency before the shot is taken
    private static final double PHASE_DELAY_SECONDS = 0.03;

    // Turret-to-target distance (meters) bounds for a valid shot
    private static final double MIN_DISTANCE = 1.5;
    private static final double MAX_DISTANCE = 6.0;

    // Empirical tables indexed by 2D turret-to-target distance (meters).
    // TODO: Replace with tuned values from on-field testing.
    private static final InterpolatingDoubleTreeMap hoodAngleDegreesMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelRPMMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightSecondsMap = new InterpolatingDoubleTreeMap();

    static {
        hoodAngleDegreesMap.put(2.18, 8.0);
        hoodAngleDegreesMap.put(2.89, 12.0);
        hoodAngleDegreesMap.put(4.30, 17.0);
        hoodAngleDegreesMap.put(5.30, 21.0);

        flywheelRPMMap.put(2.18, 2700.0);
        flywheelRPMMap.put(2.89, 3000.0);
        flywheelRPMMap.put(4.30, 3400.0);
        flywheelRPMMap.put(5.30, 3800.0);

        timeOfFlightSecondsMap.put(2.18, 0.45);
        timeOfFlightSecondsMap.put(2.89, 0.60);
        timeOfFlightSecondsMap.put(4.30, 0.85);
        timeOfFlightSecondsMap.put(5.30, 1.05);
    }

    public record ShotParameters(
        boolean isValid,
        double turretAngleDegrees,   // field-relative angle to aim the turret
        double hoodAngleDegrees,
        double flywheelRPM
    ) {}

    private ShotTrajectoryCalculator() {}

    public static ShotTrajectoryCalculator getInstance() {
        if (instance == null) instance = new ShotTrajectoryCalculator();
        return instance;
    }

    /**
     * Calculate shot parameters given the current robot state and a field target.
     *
     * @param robotPose             Estimated robot pose in the field frame
     * @param robotRelativeVelocity Robot-relative chassis speeds (from drive odometry)
     * @param target                2D field position of the scoring target
     * @return Shot parameters with hood angle, flywheel RPM, and turret aim angle
     */
    public ShotParameters calculate(Pose2d robotPose, ChassisSpeeds robotRelativeVelocity, Translation2d target) {
        // Advance the estimated pose by the phase delay using the robot-relative twist,
        // which correctly accounts for the robot's rotation during the delay.
        Pose2d phasedPose = robotPose.exp(new Twist2d(
            robotRelativeVelocity.vxMetersPerSecond * PHASE_DELAY_SECONDS,
            robotRelativeVelocity.vyMetersPerSecond * PHASE_DELAY_SECONDS,
            robotRelativeVelocity.omegaRadiansPerSecond * PHASE_DELAY_SECONDS
        ));

        // Compute turret position in the field frame by rotating the robot-relative
        // offset by the robot's heading and adding it to the robot's field position.
        Pose2d turretPose = phasedPose.transformBy(new Transform2d(ROBOT_TO_TURRET, Rotation2d.kZero));
        Translation2d turretPos = turretPose.getTranslation();

        // Compute field-relative turret velocity.
        // v_turret = v_robot_field + omega x r_turret_field
        // In 2D: v_x += -omega * r_fy,  v_y += omega * r_fx
        ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeVelocity, phasedPose.getRotation());
        double omega = robotRelativeVelocity.omegaRadiansPerSecond;
        double rTurretFieldX = turretPos.getX() - phasedPose.getX();
        double rTurretFieldY = turretPos.getY() - phasedPose.getY();
        double turretVx = fieldVelocity.vxMetersPerSecond - omega * rTurretFieldY;
        double turretVy = fieldVelocity.vyMetersPerSecond + omega * rTurretFieldX;

        // Iterative lookahead convergence: project the turret's position forward by
        // the expected time of flight, then re-evaluate the distance, repeating until
        // the estimate converges (20 iterations is sufficient in practice).
        Translation2d lookaheadTurretPos = turretPos;
        double lookaheadDistance = target.getDistance(turretPos);
        for (int i = 0; i < 20; i++) {
            double tof = timeOfFlightSecondsMap.get(lookaheadDistance);
            lookaheadTurretPos = new Translation2d(
                turretPos.getX() + turretVx * tof,
                turretPos.getY() + turretVy * tof
            );
            lookaheadDistance = target.getDistance(lookaheadTurretPos);
        }

        // Field-relative angle from the lookahead turret position toward the target
        double turretAngleDegrees = target.minus(lookaheadTurretPos).getAngle().getDegrees();

        DogLog.log("ShotCalculator/lookaheadTurretX", lookaheadTurretPos.getX());
        DogLog.log("ShotCalculator/lookaheadTurretY", lookaheadTurretPos.getY());
        DogLog.log("ShotCalculator/lookaheadDistance", lookaheadDistance);

        return new ShotParameters(
            lookaheadDistance >= MIN_DISTANCE && lookaheadDistance <= MAX_DISTANCE,
            turretAngleDegrees,
            hoodAngleDegreesMap.get(lookaheadDistance),
            flywheelRPMMap.get(lookaheadDistance)
        );
    }
}
