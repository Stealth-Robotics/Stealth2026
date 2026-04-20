package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoStep {
    private final Command command;
    private final String pathName;

    public AutoStep(Command command) {
        this.command = command;
        this.pathName = null;
    }

    public AutoStep(String pathName) {
        this.command = null;
        this.pathName = pathName;
    }

    public Command getCommand() {
        return command;
    }

    public String getPathName() {
        return pathName;
    }

    public boolean isPathFollowingCommand() {
        return pathName != null;
    }
}