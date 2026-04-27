package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.Autos.AutoName;
import frc.robot.lib.BLine.FollowPath.Builder;
import frc.robot.lib.BLine.Path;

public class AutoRoutineBuilder {
    private final Builder pathBuilder;
    private final AutoName leftName;
    private final AutoName rightName;
    private final boolean doNotFlip;
    private final HashMap<AutoName, Command> cache;
    private final HashMap<AutoName, Rotation2d> directionsCache; // ADD THIS
    private final List<AutoElement> autoComposition = new ArrayList<>();
    private final AutoSide[] sideOrder = {AutoSide.LEFT, AutoSide.RIGHT};

    // Constructor for mirrored autos (left + right)
    public AutoRoutineBuilder(AutoName leftName, AutoName rightName, Builder pathBuilder,
            HashMap<AutoName, Command> cache, HashMap<AutoName, Rotation2d> directionsCache) {
        this.pathBuilder = pathBuilder;
        this.leftName = leftName;
        this.rightName = rightName;
        this.cache = cache;
        this.directionsCache = directionsCache;
        doNotFlip = false;
    }

    // Constructor for single (non-flipped) autos
    public AutoRoutineBuilder(AutoName name, Builder pathBuilder,
            HashMap<AutoName, Command> cache, HashMap<AutoName, Rotation2d> directionsCache) {
        this.pathBuilder = pathBuilder;
        this.leftName = name;
        this.rightName = null;
        this.cache = cache;
        this.directionsCache = directionsCache;
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

    public PathElement getFirstPathElement() {
        for (AutoElement element : autoComposition) {
            if (element instanceof PathElement)
                return (PathElement) element;
        }

        return null;
    }

    public void build() {
        PathElement firstPath = getFirstPathElement();

        for (AutoSide side : sideOrder) {
            AutoName name = (side.equals(AutoSide.LEFT)) ? leftName : rightName;
            if (name == null) break;

            // Build the command as before
            SequentialCommandGroup autoCommand = new SequentialCommandGroup();
            for (AutoElement autoStep : autoComposition) {
                autoCommand.addCommands(autoStep.build(side, doNotFlip, pathBuilder));
            }

            cache.put(name, autoCommand);

            // Populate initial module direction from the first path in this routine
            if (firstPath != null && directionsCache != null) {
                try {
                    // Resolve the actual path name accounting for side flipping
                    String resolvedName = firstPath.build(side, doNotFlip, pathBuilder).getName();
                    Rotation2d dir = new Path(resolvedName).getInitialModuleDirection();
                    directionsCache.put(name, dir);
                } catch (Exception e) {
                    DogLog.log("Path detection", false);
                }
            }
        }
    }
}