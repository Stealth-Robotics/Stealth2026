package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath.Builder;
import frc.robot.lib.BLine.Path;

public class PathElement implements AutoElement {
    private final String pathName;

    public PathElement(String pathName) {
        this.pathName = pathName;
    }

    @Override
    public Command build(AutoSide side, boolean doNotFlip, Builder builder) {
        Path p = new Path(pathName);
        if (!doNotFlip && side == AutoSide.LEFT)
            p.mirror();
        return builder.build(p);
    }
}