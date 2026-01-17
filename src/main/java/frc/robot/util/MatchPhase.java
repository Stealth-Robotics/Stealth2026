package frc.robot.util;

public enum MatchPhase {
    UNDEFINED(0, 0, null),
    ENDGAME(30, 0, null),
    SHIFT4(55, 30, ENDGAME),
    SHIFT3(80, 55, SHIFT4),
    SHIFT2(105, 80, SHIFT3),
    SHIFT1(130, 105, SHIFT2),
    TRANSITION_SHIFT(140, 130, SHIFT1),
    AUTO(20, 0, TRANSITION_SHIFT);

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