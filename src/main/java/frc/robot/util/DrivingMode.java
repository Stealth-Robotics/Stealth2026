package frc.robot.util;

public enum DrivingMode {
    NORMAL(1.0),
    PRECISION(0.4);

    /**
     * Allows us to slow down when performing certain actions like shooting or climbing
     */
    private final double slowingFactor;

    DrivingMode(double slowingFactor) {
        this.slowingFactor = slowingFactor;
    }

    public double getSlowingFactor() {
        return slowingFactor;
    }
}