package frc.robot;

import java.util.HashMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
            new PIDController(6.0, 0.0, 0.0), // Translation PID
            new PIDController(4.0, 0.0, 0.0), // Rotation PID
            new PIDController(2.0, 0.0, 0.0)  // Cross-track PID
        )
        .withDefaultShouldFlip()
        .withPoseReset(drive::resetPose);

        //Cache paths
        buildDoubleBump(AutoSide.LEFT);
        buildDoubleBump(AutoSide.RIGHT);
        buildDoubleTrench(AutoSide.LEFT);
        buildDoubleTrench(AutoSide.RIGHT);

        build1729BurnsFinalsTiebreaker(AutoSide.LEFT);
        build1729BurnsFinalsTiebreaker(AutoSide.RIGHT);

        // FollowPath.registerEventTrigger("intake", deployAndIntake());
        // FollowPath.registerEventTrigger("spinup", spinupShooter());
        // FollowPath.registerEventTrigger("shoot", startShooting());
        // FollowPath.registerEventTrigger("stop_shooting", stopShooting());
    }

    public Command getAuto(String name) {
        return autoCache.get(name);
    }

    public void build1729BurnsFinalsTiebreaker(AutoSide side) {
        int shootTime = 15;

        String autoName = (side.equals(AutoSide.LEFT)) ? "LeftCompatibleBump" : "RightCompatibleBump";

        Path path = new Path("CompatibleBump");
        Path path2 = new Path();

        if (side.equals(AutoSide.LEFT)) {
            shootTime = 6;
            path2 = new Path("CompatibleBumpExtension");
        } 
        else {
            path.mirror();
        }

        FollowPath compatibleBump = pathBuilder.build(path);
        FollowPath depotSweep = pathBuilder.build(path2);

        Command autoRoutine = new SequentialCommandGroup(
            new WaitCommand(0.3),
            compatibleBump.alongWith(new WaitCommand(2).andThen(deployAndIntake())),
            shootForTime(shootTime),
            depotSweep.alongWith(deployAndIntake()),
            startShooting()
        );

        autoCache.put(autoName, autoRoutine);
    }

    public void buildDoubleTrench(AutoSide side) {
        String autoName = (side.equals(AutoSide.LEFT)) ? "LeftDoubleTrench" : "RightDoubleTrench";

        Path path = new Path("DoubleTrench_1");
        Path path2 = new Path("DoubleTrench_2");

        if (side.equals(AutoSide.LEFT)) {
            path.mirror();
            path2.mirror();
        }

        FollowPath cycle1 = pathBuilder.build(path);
        FollowPath cycle2 = pathBuilder.build(path2);

        Command autoRoutine = new SequentialCommandGroup(
            cycle1,
            shootForTime(5),
            deployAndIntake(),
            cycle2,
            startShooting()
        );

        autoCache.put(autoName, autoRoutine);
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
            cycle1.alongWith(new WaitCommand(0.5).andThen(deployAndIntake())),
            shootForTime(4.2),
            deployAndIntake(),
            cycle2
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

    private Command shootForTime(double seconds) {
        return new ParallelDeadlineGroup(
            new WaitCommand(seconds),
            startShooting()
        ).andThen(stopShooting());
    }

    private Command startShooting() {
        return shooter.shoot().alongWith(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitCommand(4),
                    intake.quickAgitate(() -> 0.6).andThen(new WaitCommand(0.25)).repeatedly()
                ),
                intake.fullAgitate()
            )
        );
    }

    private Command stopShooting() {
        return shooter.stopShooting().andThen(intake.stopRollers());
    }
}