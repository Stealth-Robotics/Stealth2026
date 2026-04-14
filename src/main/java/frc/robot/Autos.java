package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.FollowPath.Builder;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.util.AutoSide;

public class Autos {
    private Builder pathBuilder;
    private final HashMap<String, Command> autoCache = new HashMap<>();

    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;

    private final double SHOOTER_SPINUP_RPMS = 2900;

    public Autos(DriveSubsystem drive, IntakeSubsystem intake, ShootingSuperstructure shooter) {
        this.intake = intake;
        this.shooter = shooter;

        pathBuilder = new FollowPath.Builder(
            drive,
            drive::getPose,
            drive::getRobotRelativeVelocity,
            drive::applyRobotRelativeSpeeds,
            new PIDController(5.0, 0.0, 0.0), // Translation PID
            new PIDController(3.0, 0.0, 0.0), // Rotation PID
            new PIDController(2.0, 0.0, 0.0)  // Cross-track PID
        )
        .withDefaultShouldFlip()
        .withPoseReset(drive::resetPose);

        //Register event triggers
        FollowPath.registerEventTrigger("Intake", deployAndIntake());
        FollowPath.registerEventTrigger("Spinup", spinupShooter());

        //Cache paths
        buildDebugAuto();

        buildDoubleBump(AutoSide.LEFT);
        buildDoubleBump(AutoSide.RIGHT);
    }

    public Command getAuto(String name) {
        return autoCache.get(name);
    }

    public void buildDoubleBump(AutoSide side) {
        String autoName = (side.equals(AutoSide.LEFT)) ? "LeftDoubleBump" : "RightDoubleBump";

        Path path = new Path("DoubleBump_1");
        Path path2 = new Path("DoubleBump_2");

        if (side.equals(AutoSide.LEFT)) {
            path.mirror();
            path2.mirror();
        }

        FollowPath cycle1 = pathBuilder.build(path);
        FollowPath cycle2 = pathBuilder.build(path2);

        Command autoRoutine = new SequentialCommandGroup(
            deployAndIntake(),
            cycle1,
            new ParallelDeadlineGroup(new WaitCommand(5), startShooting()),
            stopShooting(),
            cycle2,
            startShooting()
        );

        autoCache.put(autoName, autoRoutine);
    }

    public void buildDebugAuto() {
        String autoName = "Debug";

        FollowPath path = pathBuilder.build(new Path("Debug"));

        Command autoRoutine = new SequentialCommandGroup(
            deployAndIntake(),
            path,
            new ParallelDeadlineGroup(new WaitCommand(3), startShooting()),
            stopShooting(),
            retractAndStopIntake()
        );

        autoCache.put(autoName, autoRoutine);
    }

    // AUTO COMMAND HELPERS

    private Command deployAndIntake() {
        return intake.deployCommand().andThen(intake.startRollers());
    }

    private Command retractAndStopIntake() {
        return intake.retractCommand().andThen(intake.stopRollers());
    }

    private Command spinupShooter() {
        return shooter.spinUp(SHOOTER_SPINUP_RPMS);
    }

    private Command startShooting() {
        return shooter.shoot();
        // .andThen(new ScheduleCommand(
        //     new SequentialCommandGroup(
        //         intake.partialAgitate(() -> 0.3),
        //         new WaitCommand(1),
        //         intake.partialAgitate(() -> 0.4).repeatedly()
        //     )
        // ));
    }

    private Command stopShooting() {
        return shooter.stopShooting().andThen(intake.stopRollers());
    }
}