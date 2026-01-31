package frc.robot.util;

public class ShooterCalc {
    private static final double g = -32.2; // g = 32.2 ft/s^2
    private static final double LATENCY_COMPENSATION = 0.040; // Estimated 40 ms of latency

    private static double shootVelocity = 0;
    private static double turretAngle = 0;
    private static double hoodAngle = 0;

    public static void update(Vector3d ballPose, Vector3d botVelo, Vector3d goalPose, double t) {

        // Account for latency by adding a bit to ball position
        Vector3d ballPoseFuture = new Vector3d(ballPose.x + botVelo.x * LATENCY_COMPENSATION, ballPose.y,
                ballPose.z + botVelo.z * LATENCY_COMPENSATION);

        // Calculated launch velocity vector - will land in goal at time t
        Vector3d ballVelo = new Vector3d(
                (goalPose.x - ballPoseFuture.x) / t - botVelo.x,
                (goalPose.y - ballPoseFuture.y) / t + g * t / 2,
                (goalPose.z - ballPoseFuture.z) / t - botVelo.z
        );

        // Update values to match velocity vector
        shootVelocity = ballVelo.magnitude();
        turretAngle = Math.atan(ballVelo.z / ballVelo.x);
        hoodAngle = Math.atan(ballVelo.y / ballVelo.x * Math.cos(turretAngle));
    }

    public double getShootVelocity() { return shootVelocity; }
    // 2 is wheel radius (inches)
    public double getShootRPM() { return shootVelocity * 30 / (Math.PI * 2 / 12); }
    public double getTurretAngle() { return turretAngle; }
    public double getHoodAngle() { return hoodAngle; }
}
