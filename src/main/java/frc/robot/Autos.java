package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;

public class Autos {
    private FollowPath.Builder pathBuilder;

    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;

    private final Command nothingAuto = new InstantCommand();

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
    }

    /**
     * Schedules the call to preload trajectory for the selected auto routine to prevent lag
     **/
    public void preloadAuto(String autoName) {
    }

    public Command debugAuto() {
        String pathName = "Debug";

        FollowPath path = pathBuilder.build(new Path(pathName));

        Command autoRoutine = new SequentialCommandGroup(
            deployAndIntake(),
            path,
            shootForTime(3),
            retractAndStopIntake()
        );

        return (pathName.isBlank()) ? nothingAuto : autoRoutine;
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