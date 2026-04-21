package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath.Builder;

public class CommandElement implements AutoElement {
    private final Supplier<Command> command;

    public CommandElement(Supplier<Command> command) {
        this.command = command;
    }

    @Override
    public Command build(AutoSide side, boolean doNotFlip, Builder builder) {
        return command.get();
    }
}
