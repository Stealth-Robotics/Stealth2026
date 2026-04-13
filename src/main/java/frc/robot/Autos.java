package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;

public class Autos {
    private final AutoFactory autoFactory;

    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;

    private final AutoRoutine nothingAuto;

    private final double SHOOTER_SPINUP_RPMS = 2900;

    public Autos(AutoFactory autoFactory, IntakeSubsystem intake, ShootingSuperstructure shooter) {
        this.autoFactory = autoFactory;
        
        this.intake = intake;
        this.shooter = shooter;

        nothingAuto = autoFactory.newRoutine("nothing");
    }

    /**
     * Schedules the call to preload trajectory for the selected auto routine to prevent lag
     **/
    public void preloadAuto(String autoName) {
        var trajectory = autoFactory.cache().loadTrajectory(autoName).orElse(null);
        if (trajectory != null) 
            autoFactory.resetOdometry(trajectory);
    }

    public AutoRoutine debugAuto() {
        String pathName = "Debug";

        AutoRoutine routine = autoFactory.newRoutine("routine");

        AutoTrajectory path = routine.trajectory(pathName, 0);
        path.atTime("Intake").onTrue(deployAndIntake());
        path.atTime("Spinup").onTrue(spinupShooter());
        path.atTime("Shoot").onTrue(startShooting());
        
        AutoTrajectory path2 = routine.trajectory(pathName, 1);
        path2.atTime("Intake2").onTrue(deployAndIntake());
        path2.atTime("Spinup2").onTrue(spinupShooter());
        path2.atTime("Shoot2").onTrue(startShooting());

        routine.active().onTrue(path.cmd());

        path.done().onTrue(
            new SequentialCommandGroup(
                new WaitCommand(3),
                intake.fullAgitate().withTimeout(2),
                stopShooting(),
                path2.cmd()
            )
        );

        return (pathName.isBlank()) ? nothingAuto : routine;
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
        return shooter.shoot().alongWith(
            new SequentialCommandGroup(
                intake.partialAgitate(() -> 0.5), //One quick agitate to start the ball rolling
                new WaitCommand(1.5),
                intake.partialAgitate(() -> 0.5).andThen(new WaitCommand(0.2)).repeatedly()
            )
        );
    }

    private Command stopShooting() {
        return shooter.stopShooting().andThen(intake.stopRollers());
    }
}