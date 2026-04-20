package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.Autos.AutoName;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.FollowPath.Builder;

public class AutoRoutineBuilder {
    private final Builder pathBuilder;

    private final AutoName leftName;
    private final AutoName rightName;

    private final HashMap<AutoName, Command> cache;
    private final List<AutoStep> autoComposition = new ArrayList<>();

    private final AutoSide[] sideOrder = {AutoSide.LEFT, AutoSide.RIGHT};

    public AutoRoutineBuilder(AutoName leftName, AutoName rightName, Builder pathBuilder, HashMap<AutoName, Command> cache) {
        this.pathBuilder = pathBuilder;
        this.leftName = leftName;
        this.rightName = rightName;
        this.cache = cache;
    }

    public AutoRoutineBuilder(AutoName name, Builder pathBuilder, HashMap<AutoName, Command> cache) {
        this.pathBuilder = pathBuilder;
        this.leftName = name;
        this.rightName = null;
        this.cache = cache;
    }

    public AutoRoutineBuilder followPath(String pathName) {
        autoComposition.add(new AutoStep(pathName));
        return this;
    }

    public AutoRoutineBuilder addCommands(Command command) {
        autoComposition.add(new AutoStep(command));
        return this;
    }

    public void build() {
        for (AutoSide side : sideOrder) {
            AutoName name = (side.equals(AutoSide.LEFT)) ? leftName : rightName;

            if (name == null)
                break;

            List<Command> autoCommand = new ArrayList<>();

            for (AutoStep autoStep : autoComposition) {
                if (autoStep.isPathFollowingCommand()) {
                    Path p = new Path(autoStep.getPathName());
                    if (side.equals(AutoSide.RIGHT))
                        p.mirror();

                    autoCommand.add(pathBuilder.build(p));
                }
                else {
                    autoCommand.add(autoStep.getCommand());
                }
            }

            cache.put(name, new SequentialCommandGroup(autoCommand.toArray(new Command[0])));
        }
    }
}