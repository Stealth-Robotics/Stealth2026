package frc.robot.util;

import dev.doglog.DogLog;

public class DogLogUtil {
    public static final long MOTOR_LOGGING_INTERVAL_MS = 250;
    public static final long LIMELIGHT_LOGGING_INTERVAL = 200;
    public static final long STATS_LOGGING_INTERVAL = 200;
    
    //Logs a double to two decimal places
    public static void logDouble(String key, double value) {
        DogLog.log(key, Math.round(value * 100.0) / 100.0);
    }
}
