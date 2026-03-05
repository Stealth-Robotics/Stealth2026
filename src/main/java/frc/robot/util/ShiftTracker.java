package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

/**
 * Ultility class that keeps track of who can score when and for how long (dependent on who won auto)
 */
public class ShiftTracker {
    private static MatchPhase phase = MatchPhase.UNKNOWN;
    private static final Timer matchTimer = new Timer();

    private static Alliance ourAlliance = null;
    private static Alliance allianceThatWonAuto = null;

    /* This constant stands for the amount of time that you can shoot before it's legal
     * And still have the fuel count in the hub
    */
    private static final double SCORING_DELAY_SECONDS = 0.8;

    //Just in case we start teleop without auto and we need to correctly offset the time
    private static double timeOffset = 0;

    public static void start() {
        reset();

        if (DriverStation.isAutonomous())
            phase = MatchPhase.AUTO;
        else {
            phase = MatchPhase.TRANSITION_SHIFT;

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

    public static MatchPhase getCurrentMatchPhase() {
        return phase;
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
            if (phase != MatchPhase.UNKNOWN && phase != null) {
                if (matchTimer.hasElapsed(phase.getEndTime()))
                    phase = phase.getNext();
            }
            else {
                /* Phase becomes null when we are in auto or teleop DS modes and have exceeded the standard
                * match duration. We stop tracking shifts and any calls to canScore() will return true for testing purposes
                */
                reset();
                phase = MatchPhase.UNKNOWN;
            }
        }
    }

    /**
     * @return Whether or not we can score into our alliance's hub
     */
    public static boolean canScore() {
        if (!matchTimer.isRunning())
            return true;
        else if (ourAlliance == null)
            return false;

        switch (phase) {
            case AUTO -> { return true; }

            case AUTO_TELE_TRANSITION -> {
                if (!weWonAuto() && getTimeLeftInShift() <= SCORING_DELAY_SECONDS) return true;
                else return false;
            }

            case TRANSITION_SHIFT -> {
                if (weWonAuto()) return getTimeLeftInShift() >= SCORING_DELAY_SECONDS;
                else return true;
            }

            case SHIFT1, SHIFT3 -> {
                if (weWonAuto()) return getTimeLeftInShift() <= SCORING_DELAY_SECONDS;
                else return getTimeLeftInShift() >= SCORING_DELAY_SECONDS;
            }

            case SHIFT2, SHIFT4 -> {
                if (weWonAuto()) return getTimeLeftInShift() >= SCORING_DELAY_SECONDS;
                else return getTimeLeftInShift() <= SCORING_DELAY_SECONDS;
            }

            case ENDGAME, UNKNOWN -> { return true; }
            default -> { return false; }
        }
    }

    public static boolean isRunning() {
        return matchTimer.isRunning();
    }

    private static boolean weWonAuto() {
        return ourAlliance == allianceThatWonAuto;
    }

    public static double getTime() {
        return matchTimer.get();
    }

    public static double getTimeLeftInShift() {
        return Math.max(0, phase.getEndTime() - (matchTimer.get() + timeOffset));
    }
}