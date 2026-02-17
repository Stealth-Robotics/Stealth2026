package frc.robot.util;

public enum MatchPhase {
    UNKNOWN(0, 0, null),
    ENDGAME(133, 163, UNKNOWN),
    SHIFT4(108, 133, ENDGAME),
    SHIFT3(83, 108, SHIFT4),
    SHIFT2(58, 83, SHIFT3),
    SHIFT1(33, 58, SHIFT2),
    TRANSITION_SHIFT(23, 33, SHIFT1),
    AUTO_TELE_TRANSITION(20, 23, TRANSITION_SHIFT),
    AUTO(0, 20, AUTO_TELE_TRANSITION);

    private final int START_TIME_SECONDS;
    private final int END_TIME_SECONDS;

    private final MatchPhase NEXT_PHASE;

    MatchPhase(int startTime, int endTime, MatchPhase next) {
        START_TIME_SECONDS = startTime;
        END_TIME_SECONDS = endTime;
        NEXT_PHASE = next;
    }

    public MatchPhase getNext() {
        return NEXT_PHASE;
    }

    public int getStartTime() {
        return START_TIME_SECONDS;
    }

    public int getEndTime() {
        return END_TIME_SECONDS;
    }
}