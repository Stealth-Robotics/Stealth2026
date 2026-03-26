package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Timer;

/**
 * Ultility class that keeps track of who can score when and for how long (dependent on who won auto)
 */
public class ShiftTracker {
    private static MatchPhase phase = MatchPhase.UNKNOWN;
    private static final Timer matchTimer = new Timer();

    private static Alliance ourAlliance = null;
    private static Alliance allianceThatWonAuto = null;

    //Just in case we start teleop without auto and we need to correctly offset the time
    private static double timeOffset = 0;

    public static Trigger hubIsActive = new Trigger(() -> {
        return hubIsActive();
    });

    public static Trigger hubIsInactive = new Trigger(() -> {
        return !hubIsActive();
    });

    public static Trigger shiftWarningTrigger = new Trigger(() -> {
        return
            phase != MatchPhase.AUTO &&
            phase != MatchPhase.AUTO_TELE_TRANSITION &&
            phase != MatchPhase.ENDGAME &&
            getTimeLeftInShift() <= 5;
    });

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
                if (getTime() > phase.getEndTime())
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
    public static boolean hubIsActive() {
        switch (phase) {
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

    public static boolean hubIsActiveNextShift() {
        if (phase.equals(MatchPhase.TRANSITION_SHIFT))
            return !weWonAuto();
        else return !hubIsActive();
    }

    public static boolean isRunning() {
        return matchTimer.isRunning();
    }

    public static boolean weWonAuto() {
        if (allianceThatWonAuto == null)
            return true;
        return ourAlliance == allianceThatWonAuto;
    }

    public static double getTimeLeftInShift() {
        return Math.max(0, phase.getEndTime() - getTime());
    }

    private static double getTime() {
        return matchTimer.get() + timeOffset;
    }
}