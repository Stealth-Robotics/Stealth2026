package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath.Builder;

public interface AutoElement {
    Command build(AutoSide side, Builder builder);
}