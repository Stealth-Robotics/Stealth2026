package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.Autos.AutoName;
import frc.robot.lib.BLine.FollowPath.Builder;

public class AutoRoutineBuilder {
    private final Builder pathBuilder;

    private final AutoName leftName;
    private final AutoName rightName;

    private final boolean doNotFlip;

    private final HashMap<AutoName, Command> cache;
    private final List<AutoElement> autoComposition = new ArrayList<>();

    private final AutoSide[] sideOrder = {AutoSide.LEFT, AutoSide.RIGHT};

    public AutoRoutineBuilder(AutoName leftName, AutoName rightName, Builder pathBuilder, HashMap<AutoName, Command> cache) {
        this.pathBuilder = pathBuilder;
        this.leftName = leftName;
        this.rightName = rightName;
        this.cache = cache;

        doNotFlip = false;
    }

    public AutoRoutineBuilder(AutoName name, Builder pathBuilder, HashMap<AutoName, Command> cache) {
        this.pathBuilder = pathBuilder;
        this.leftName = name;
        this.rightName = null;
        this.cache = cache;

        doNotFlip = true;
    }

    public AutoRoutineBuilder followPath(String pathName) {
        autoComposition.add(new PathElement(pathName));
        return this;
    }

    public AutoRoutineBuilder addCommand(Supplier<Command> command) {
        autoComposition.add(new CommandElement(command));
        return this;
    }

    public void build() {
        for (AutoSide side : sideOrder) {
            AutoName name = (side.equals(AutoSide.LEFT)) ? leftName : rightName;

            if (name == null)
                break;

            SequentialCommandGroup autoCommand = new SequentialCommandGroup();
            for (AutoElement autoStep : autoComposition) {
                autoCommand.addCommands(autoStep.build(side, doNotFlip, pathBuilder));
            }
            cache.put(name, autoCommand);
        }
    }
}