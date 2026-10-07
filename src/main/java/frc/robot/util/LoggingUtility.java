package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;

public final class LoggingUtility {

    private LoggingUtility() {}

    private static final int LOW_PRIORITY_INTERVAL_MS = 250;
    private static final int HIGH_PRIORITY_INTERVAL_MS = 50;

    //Subsystem log settings
    public static final boolean LOG_DRIVE = false;
    public static final boolean LOG_INTAKE = false;
    public static final boolean LOG_SHOOTER = false;
    public static final boolean LOG_TRANSFER = false;
    public static final boolean LOG_TURRET = false;

    //Device log settings
    public static final boolean LOG_PIGEON = false;
    public static final boolean LOG_CAN = false;
    public static final boolean LOG_LIMELIGHTS = false;
    public static final boolean LOG_PDH = false;

    private static double lastLowPriorityLog = 0.0;
    private static double lastHighPriorityLog = 0.0;

    private static boolean lowPriorityDue = false;
    private static boolean highPriorityDue = false;

    //Logs a double to two decimal places
    public static void logDouble(String key, double value) {
        DogLog.log(key, Math.round(value * 100.0) / 100.0);
    }

    //Logs a double to two decimal places and forces it to display on NetworkTables
    public static void logDoubleForceNT(String key, double value) {
        DogLog.forceNt.log(key, Math.round(value * 100.0) / 100.0);
    }

    //Signals when to update all the low priority logs
    public static boolean updateLowPriorityLogs() {
        return lowPriorityDue;
    }

    //Signals when to update all the high priority logs
    public static boolean updateHighPriorityLogs() {
        return highPriorityDue;
    }

    public static void update() {
        double now = Timer.getFPGATimestamp();

        lowPriorityDue = (now - lastLowPriorityLog) >= LOW_PRIORITY_INTERVAL_MS / 1000.0;
        highPriorityDue = (now - lastHighPriorityLog) >= HIGH_PRIORITY_INTERVAL_MS / 1000.0;

        if (lowPriorityDue)
            lastLowPriorityLog = now;

        if (highPriorityDue)
            lastHighPriorityLog = now;
    }
}