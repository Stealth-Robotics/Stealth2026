package frc.robot.util;

import dev.doglog.DogLog;

public class DogLogUtil {
    public static final long MOTOR_LOGGING_INTERVAL_MS = 100;
    
    //Logs a double to two decimal places
    public static void logDouble(String key, double value) {
        DogLog.log(key, Math.round(value * 100.0) / 100.0);
    }
}
