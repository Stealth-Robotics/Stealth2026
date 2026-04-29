package frc.robot.auto;

import java.util.HashMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.FollowPath.Builder;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;

public class Autos {
    private Builder pathBuilder;
    private final HashMap<AutoName, Command> autoCache = new HashMap<>();

    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;

    private final double SHOOTER_SPINUP_RPMS = 2900;

    public enum AutoName {
        LEFT_DOUBLE_BUMP,
        RIGHT_DOUBLE_BUMP,

        LEFT_DOUBLE_TRENCH,
        RIGHT_DOUBLE_TRENCH,

        COMPATIBLE_BUMP_DEPOT,

        COMPATIBLE_BUMP_DEPOT_MID,

        LEFT_TRENCH_STEAL
    }

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

        // FollowPath.registerEventTrigger("shoot", shoot());
        // FollowPath.registerEventTrigger("stop_shoot", stopShooting());
        FollowPath.registerEventTrigger("intake", deployAndIntake());

        //Build autos
        buildDoubleBump();
        buildDoubleTrench();
        buildCompatibleBumpDepot();
        buildCompatibleBumpDepotMid();
        buildLeftTrenchSteal();
    }

    public HashMap<AutoName, Command> getAutoLibrary() {
        return autoCache;
    }

    public void buildLeftTrenchSteal() {
        var autoBuilder = new AutoRoutineBuilder(
            AutoName.LEFT_TRENCH_STEAL,
            pathBuilder,
            autoCache
        );

        autoBuilder
            .addCommand(() -> new WaitCommand(2))
            .followPath("LeftTrenchSteal")
            .addCommand(() -> shootWithAgitate())
            .build();
    }

    /*
     * Starts on the left bump, does one mid cycle, shoots and then grabs the depot while shooting
     * https://www.youtube.com/watch?v=FGAFQMbGrpY
     */
    public void buildCompatibleBumpDepot() {
        var autoBuilder = new AutoRoutineBuilder(
            AutoName.COMPATIBLE_BUMP_DEPOT,
            pathBuilder,
            autoCache
        );

        autoBuilder
            .addCommand(() -> new WaitCommand(0.25)) //Starting delay
            .followPath("CompatibleBump")
            .addCommand(() -> shootForTime(5))
            .addCommand(() -> deployAndIntake())
            .followPath("CompatibleBumpExtension")
            .addCommand(() -> shootWithAgitate())
            .build();
    }

    public void buildCompatibleBumpDepotMid() {
        var autoBuilder = new AutoRoutineBuilder(
            AutoName.COMPATIBLE_BUMP_DEPOT_MID, 
            pathBuilder, 
            autoCache
        );

        autoBuilder
            .addCommand(() -> new WaitCommand(0.25))
            .followPath("CompatibleBump2")
            .addCommand(() -> shootForTime(5))
            .addCommand(() -> deployAndIntake())
            .followPath("CompatibleBumpExtension")
            .addCommand(() -> shootWithAgitate())
            .build();            
    }

    public void buildDoubleTrench() {
        var autoBuilder = new AutoRoutineBuilder(
            AutoName.LEFT_DOUBLE_TRENCH,
            AutoName.RIGHT_DOUBLE_TRENCH,
            pathBuilder,
            autoCache
        );

        autoBuilder
            .followPath("DoubleTrench_1")
            .addCommand(() -> shootForTime(5))
            .addCommand(() -> deployAndIntake())
            .followPath("DoubleTrench_2")
            .addCommand(() -> shootWithAgitate())
            .build();
    }

    public void buildDoubleBump() {
        var autoBuilder = new AutoRoutineBuilder(
            AutoName.LEFT_DOUBLE_BUMP,
            AutoName.RIGHT_DOUBLE_BUMP,
            pathBuilder,
            autoCache
        );

        autoBuilder
            .followPath("DoubleBump_1")
            .addCommand(() -> shootForTime(4))
            .addCommand(() -> deployAndIntake())
            .followPath("DoubleBump_2")
            .addCommand(() -> shootWithAgitate())
            .build();
    }

    // AUTO COMMAND HELPERS

    private Command deployAndIntake() {
        return new ScheduleCommand(intake.deployCommand().andThen(intake.startRollers()));
    }

    private Command retractAndStopIntake() {
        return new ScheduleCommand(intake.retractCommand().andThen(intake.stopRollers()));
    }

    private Command spinupShooter() {
        return new ScheduleCommand(shooter.spinUp(SHOOTER_SPINUP_RPMS));
    }

    private Command shootForTime(double seconds) {
        return new SequentialCommandGroup(
            shootWithAgitate(),
            new WaitCommand(seconds),
            stopShooting()
        );
    }

    private Command shootWithAgitate() {
        return new ScheduleCommand(shooter.shoot().alongWith(
            new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                    new WaitCommand(4),
                    intake.quickAgitate(() -> 0.6).andThen(new WaitCommand(0.25)).repeatedly()
                ),
                intake.fullAgitate()
            )
        ));
    }

    private Command shoot() {
        return new ScheduleCommand(shooter.shoot());
    }

    private Command stopShooting() {
        return new ScheduleCommand(shooter.stopShooting().andThen(intake.stopRollers()));
    }
}