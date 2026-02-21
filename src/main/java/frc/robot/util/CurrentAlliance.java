package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class CurrentAlliance {
    private static Alliance latestAlliance = Alliance.Blue;

    public static Alliance get() {
        return latestAlliance;
    }

    public static void update() {
        latestAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    }
}
