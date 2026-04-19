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
            new PIDController(10.0, 0.0, 0.0), // Translation PID
            new PIDController(5.0, 0.0, 0.0), // Rotation PID
            new PIDController(2.0, 0.0, 0.0)  // Cross-track PID
        )
        .withDefaultShouldFlip()
        .withPoseReset(drive::resetPose);

        //Cache paths
        buildDoubleBump(AutoSide.LEFT);
        buildDoubleBump(AutoSide.RIGHT);
        buildDoubleTrench(AutoSide.LEFT);
        buildDoubleTrench(AutoSide.RIGHT);
        buildMiddleBump(AutoSide.LEFT);
        buildMiddleBump(AutoSide.RIGHT);
        buildDepotSweep();
        buildCenterDepot();
        buildCenterDepotPlusPass();
        build1729BurnsFinalsTiebreaker(AutoSide.LEFT);
        build1729BurnsFinalsTiebreaker(AutoSide.RIGHT);

        FollowPath.registerEventTrigger("", startShooting());
        FollowPath.registerEventTrigger("Deploy", deployAndIntake().asProxy());
        FollowPath.registerEventTrigger("Retract", retractAndStopIntake().asProxy());
        FollowPath.registerEventTrigger("Shoot", startShooting().asProxy());
        FollowPath.registerEventTrigger("StopShooting", stopShooting().asProxy());
        FollowPath.registerEventTrigger("SpinUp", spinupShooter().asProxy());
    }

    public Command getAuto(String name) {
        return autoCache.get(name);
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
            shootForTime(5),
            deployAndIntake(),
            cycle2,
            startShooting()
        );

        autoCache.put(autoName, autoRoutine);
    }

    // Risky steal auto. Drives from the hub to the side trench, before sweeping opposing alliance's hub
    // around 6-7 seconds (generally the most safe). Returns on the left side to clear the depot.
    // EXTREMELY RISKY!!
    public void buildMiddleBump(AutoSide side) {
        String autoName = (side.equals(AutoSide.LEFT)) ? "LeftMiddleBump" : "RightMiddleBump";

        Path path = (side.equals(AutoSide.LEFT)) ? new Path("CenterBumpAutoLeft") : new Path("CenterBumpAutoRight");
        Path path2 = new Path("DepotSweep");

        FollowPath preDepot = pathBuilder.build(path);
        FollowPath depotSweep = pathBuilder.build(path2);


        Command autoRoutine = new SequentialCommandGroup(
            preDepot,
            depotSweep.alongWith(deployAndIntake().asProxy()),
            startShooting()
        );

        autoCache.put(autoName, autoRoutine);
    }

    // Depot auto that begins by the left bump. Compatible with 2 generic side sweepers.
    public void buildDepotSweep() {
        String autoName = "DepotSweep";

        Path path = new Path("DepotSweep");

        FollowPath depotSweep = pathBuilder.build(path);

        Command autoRoutine = new SequentialCommandGroup(
            depotSweep.alongWith(deployAndIntake().asProxy()),
            startShooting()
        );

        autoCache.put(autoName, autoRoutine);
    }

    // A generic center depot auto with hub start. Compatible with 2 generic side sweepers.
    public void buildCenterDepot() {
        String autoName = "CenterDepot";
        
        Path path = new Path("CenterDepot");

        FollowPath centerDepot = pathBuilder.build(path);

        Command autoRoutine = new SequentialCommandGroup(
            centerDepot.alongWith(deployAndIntake().asProxy()),
            startShooting()
        );

        autoCache.put(autoName, autoRoutine);
    }

    // A compatible auto to 2 generic side 2-sweeps. Initially runs a center depot sweep path,
    // then uses final seconds of auto to pass along the center and steal the opposing team's fuel.
    public void buildCenterDepotPlusPass() {
        double PASSTHROGH_TIME = 10; // Time for robot to pass through the trench
        double CYCLE_1_TIME = 3.83; // Time of the first pass, in seconds
        double REACH_TRENCH_TIME = 1.00; // Time from the end of the first pass to reaching the trench, in seconds

        double shootingTime = PASSTHROGH_TIME - CYCLE_1_TIME - REACH_TRENCH_TIME; // Time to shoot during the first pass, in seconds

        String autoName = "CenterDepotPlus";

        FollowPath centerDepot = pathBuilder.build(new Path("CenterDepot"));
        FollowPath centerDepotExtension = pathBuilder.build(new Path("CenterDepotExtension"));

        Command autoRoutine = new SequentialCommandGroup(
            centerDepot.alongWith(deployAndIntake().asProxy()),
            shootForTime(shootingTime),
            centerDepotExtension.alongWith(stopShooting().asProxy())
        );

        autoCache.put(autoName, autoRoutine);
    }

    // From 1729's New England Burns Finals tiebreaker auto. Sweeps around 2 bots with generic
    // side paths. Uses depot in the left version.
    public void build1729BurnsFinalsTiebreaker(AutoSide side) {
        int shootTime = 15;

        String autoName = "CompatibleBump";

        Path path = new Path("CompatibleBump");
        Path path2 = new Path();

        if (side.equals(AutoSide.LEFT)) {
            path.mirror();
            shootTime = 6;
            path2 = new Path("CompatibleBumpExtension");
        }

        FollowPath compatibleBump = pathBuilder.build(path);
        FollowPath depotSweep = pathBuilder.build(path2);

        Command autoRoutine = new SequentialCommandGroup(
            compatibleBump,
            shootForTime(shootTime),
            depotSweep.alongWith(deployAndIntake().asProxy()),
            startShooting()
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
                intake.partialAgitate(() -> 0.5),
                new WaitCommand(1),
                intake.partialAgitate(() -> 0.4).repeatedly()
            )
        );
    }

    private Command stopShooting() {
        return shooter.stopShooting().andThen(intake.stopRollers());
    }
}