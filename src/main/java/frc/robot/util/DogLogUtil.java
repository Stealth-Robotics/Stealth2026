package frc.robot.util;

import dev.doglog.DogLog;

public class DogLogUtil {
    //Logs a double to two decimal places
    public static void logDouble(String key, double value) {
        DogLog.log(key, Math.round(value * 100.0) / 100.0);
    }
}
