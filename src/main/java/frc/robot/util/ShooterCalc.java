package frc.robot.util;

import edu.wpi.first.math.geometry.Translation3d;

public class ShooterCalc {
    private static final double g = 32.2; // Gravitational constant - g = 9.8 m/s^2 = 32.2 ft/s^2

    // Limits on accurate shooting
    //TODO: TUNE
    private static final double TURRET_MAX_ANGLE = 90;
    private static final double TURRET_MIN_ANGLE = 0;
    private static final double MAX_VELO = 20;
    private static final double MAX_ACCEL = 4;

    //TODO: input wheel diameter
    private static final double SHOOTER_WHEEL_CIRCUMFERENCE = 3 * Math.PI; // Circumference of the shooter wheel, in inches, for RPM calc

    //TODO: TUNE
    private static final double LATENCY_COMPENSATION = 0.040; // Estimated 40 ms of latency
    private static final double ANTIDRAG_COEFFICIENT = 1.3; // Coefficient multiplied by horizontal shot velocity to compensate for drag

    private static double shootVelocity = 0;
    private static double turretAngle = 0;
    private static double hoodAngle = 0;
    private static boolean isValid = true;

    // Updates shootVelocity, turretAngle, and hoodAngle                                                         Coordinate System:                      [Ceiling]
    // ballPose - Position of the ball in relation to the field (bot pos + turret pos on robot)                           [Opposing side]                    ^ (+Y)
    // botVelo - bot velocity in feet/second                                                                                     ^ (+Z)                      |
    // botAccel - bot acceleration in feet/second^2 for latency compensation                                                     |                           |
    // goalPose - Position of the goal in relation to the field (z forward, x right, origin doesn't matter)      [left side] <-[Bot]-> [right side]          V (-Y)
    // botRPMCCW - Rotational velocity of the bot in Counterclockwise Rotations per Minute                           (-X)                  (+X)           [Floor]
    // height - target maxHeight of the shot, in feet (~13 for shooting, ~5 for passing)
    public static void update(Translation3d ballPose, Translation3d botVelo, Translation3d botAccel, Translation3d goalPose, double botRPMCCW, double height) {

        // Account for latency by adding a bit to ball position | NOTE: does not account for acceleration
        Translation3d ballPoseFuture = new Translation3d(ballPose.getX() + botVelo.getX() * LATENCY_COMPENSATION, ballPose.getY(),
                ballPose.getZ() + botVelo.getZ() * LATENCY_COMPENSATION);

        // Account for latency by adding a bit to ball velocity | NOTE: if acceleration is too high, shots may be inaccurate
        Translation3d botVeloFuture = new Translation3d(botVelo.getX() + botAccel.getX() * LATENCY_COMPENSATION, botVelo.getY(),
                botVelo.getZ() + botAccel.getZ() * LATENCY_COMPENSATION);

        // t is calculated to be the time for the ball to reach desired height, and return to goal height, under vacuum conditions
        double t = Math.sqrt(2 * (height - ballPose.getY()) / g) + Math.sqrt(2 * (height - goalPose.getY()) / g);

        // Calculated launch velocity vector - will land in goal at time t
        Translation3d ballVelo = new Translation3d(
                ANTIDRAG_COEFFICIENT * (goalPose.getX() - ballPoseFuture.getX()) / t - botVeloFuture.getX(),
                (goalPose.getY() - ballPoseFuture.getY()) / t - g * t / 2,
                ANTIDRAG_COEFFICIENT * (goalPose.getZ() - ballPoseFuture.getZ()) / t - botVeloFuture.getZ()
        );

        // Update values to match velocity vector
        shootVelocity = ballVelo.getNorm();
        turretAngle = Math.atan(ballVelo.getZ() / ballVelo.getX());
        hoodAngle = Math.atan(ballVelo.getY() / ballVelo.getX() * Math.cos(turretAngle));

        // Account for latency in bot rotation by applying bot rotation to turret
        turretAngle -= botRPMCCW / 60 * LATENCY_COMPENSATION;

        // Check whether optimal conditions are met for a successful shot
        isValid = (botAccel.getNorm() < MAX_ACCEL && botVeloFuture.getNorm() < MAX_VELO && angleInRange(turretAngle, TURRET_MIN_ANGLE, TURRET_MAX_ANGLE));
    }
    private static boolean angleInRange(double angle, double min, double max) {
        if(max > min) return (angle < max && angle > min);
        else if (angle < min) return (angle < max && angle + 2 * Math.PI > min);
        return (angle < max + 2 * Math.PI && angle > min);
    }

    public double getShootVelocity() { return shootVelocity; }
    // RPM = [velocity]ft/s * 12 in/ft * 60 s/min * 1 rotation/[Circumference] in = 720/[Circumference]
    public double getShootRPM() { return shootVelocity * 720 / SHOOTER_WHEEL_CIRCUMFERENCE; }
    public double getTurretAngle() { return turretAngle; }
    public double getHoodAngle() { return hoodAngle; }
    public boolean validShot() { return isValid; }
}
