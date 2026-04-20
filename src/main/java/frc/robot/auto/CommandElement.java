package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath.Builder;

public class CommandElement implements AutoElement {
    private final Command command;

    public CommandElement(Command command) {
        this.command = command;
    }

    @Override
    public Command build(AutoSide side, Builder builder) {
        return command;
    }
}
