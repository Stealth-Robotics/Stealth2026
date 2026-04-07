package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Timer;

/**
 * Utility class that keeps track of who can score when and for how long (dependent on who won auto)
 */
public class ShiftTracker {
    private static MatchPhase matchPhase = MatchPhase.UNKNOWN;
    private static final Timer matchTimer = new Timer();

    private static Alliance ourAlliance = null;
    private static Alliance allianceThatWonAuto = null;

    //Just in case we start teleop without auto and we need to correctly offset the time
    private static double timeOffset = 0;

    public static Trigger shiftWarningTrigger = new Trigger(() -> {
        return
            matchPhase != MatchPhase.AUTO &&
            matchPhase != MatchPhase.AUTO_TELE_TRANSITION &&
            matchPhase != MatchPhase.ENDGAME &&
            getTimeLeftInShift() <= 6;
    });

    public static void start() {
        reset();

        if (DriverStation.isAutonomous())
            matchPhase = MatchPhase.AUTO;
        else {
            matchPhase = MatchPhase.TRANSITION_SHIFT;

            double autoDuration = (MatchPhase.AUTO.getEndTime() - MatchPhase.AUTO.getStartTime());
            double autoToTeleopDuration = (MatchPhase.AUTO_TELE_TRANSITION.getEndTime() - MatchPhase.AUTO_TELE_TRANSITION.getStartTime());
            timeOffset = autoDuration + autoToTeleopDuration;
        }

        matchTimer.start();
    }

    public static void reset() {
        matchTimer.reset();
        matchTimer.stop();

        timeOffset = 0;
    }

    private static MatchPhase getCurrentMatchPhase() {
        return matchPhase;
    }

    /**
     * Called periodically to ensure we accurately track which shift we are in
     */
    public static void update() {
        if (allianceThatWonAuto == null) {
            String speculatedAutoWinner = DriverStation.getGameSpecificMessage();
            if (!speculatedAutoWinner.isEmpty())
                allianceThatWonAuto = (speculatedAutoWinner.equals("R")) ? Alliance.Red : Alliance.Blue;
        }

        if (ourAlliance == null)
            ourAlliance = DriverStation.getAlliance().orElse(null);

        if (isRunning()) {
            if (matchPhase != null && matchPhase != MatchPhase.UNKNOWN) {
                if (getTime() > matchPhase.getEndTime())
                    matchPhase = matchPhase.getNext();
            }
            else {
                /* Phase becomes null when we are in auto or teleop DS modes and have exceeded the standard
                * match duration. We stop tracking shifts and any calls to canScore() will return true for testing purposes
                */
                reset();
                matchPhase = MatchPhase.UNKNOWN;
            }
        }

        //Logging ShiftTracker's calculations
        DogLog.forceNt.log("Match Phase", ShiftTracker.getCurrentMatchPhase());
        DogLog.forceNt.log("Shift Time Left", ShiftTracker.getTimeLeftInShift());
    }

    /**
     * @return Whether or not we can score into our alliance's hub
     */
    public static boolean hubIsActive() {
        switch (matchPhase) {
            case AUTO -> { return true; }

            case TRANSITION_SHIFT -> { return true; }

            case SHIFT1, SHIFT3 -> {
                return !weWonAuto();
            }

            case SHIFT2, SHIFT4 -> {
                return weWonAuto();
            }

            case ENDGAME, UNKNOWN -> { return true; }

            default -> { return false; }
        }
    }

    public static boolean isRunning() {
        return matchTimer.isRunning();
    }

    private static boolean weWonAuto() {
        if (allianceThatWonAuto == null || ourAlliance == null)
            return true;
        return ourAlliance == allianceThatWonAuto;
    }

    private static double getTimeLeftInShift() {
        if (!isRunning())
            return 0;
        return Math.max(0, matchPhase.getEndTime() - getTime());
    }

    private static double getTime() {
        return matchTimer.get() + timeOffset;
    }
}