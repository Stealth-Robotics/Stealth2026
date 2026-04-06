package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;

public class Autos {
    private final AutoFactory autoFactory;

    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;

    private final AutoRoutine nothingAuto;

    public Autos(AutoFactory autoFactory, IntakeSubsystem intake, ShootingSuperstructure shooter) {
        this.autoFactory = autoFactory;
        
        this.intake = intake;
        this.shooter = shooter;

        nothingAuto = autoFactory.newRoutine("nothing");
    }

    public enum AutoPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    /**
     * Schedules the call to preload trajectory for the selected auto routine to prevent lag
     **/
    public void preloadAuto(String autoName) {
        autoFactory.cache().loadTrajectory(autoName);
    }

    private Command stopShooting() {
        return shooter.spinUp(0).andThen(stopAgitating());
    }

    private Command startAgitating() {
        return new SequentialCommandGroup(
            new WaitCommand(1),
            intake.agitate().repeatedly()
        );
    }

    private Command stopAgitating() {
        //Overrides the agitating using requirements
        return new InstantCommand(() -> intake.stopCommand());
    }

    private Command deployAndIntake() {
        return intake.deployCommand()
            .andThen(intake.intakeCommand());
    }

    private Command shootCommand() {
        return shooter.shoot();
    }

    private Command spinupShooter() {
        return shooter.spinUp(2500);
    }

    /*
     * Delayed middle steal auto
     */
    public AutoRoutine middle(AutoPosition position) {
        String pathName = switch (position) {
            case LEFT -> "LeftMiddle";
            case RIGHT -> "RightMiddle";
            default -> "";
        };

        if (pathName.isBlank())
            return nothingAuto;

        AutoRoutine routine = autoFactory.newRoutine("routine");

        AutoTrajectory path = routine.trajectory(pathName, 0);

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                shootCommand(),
                new WaitCommand(5), //Wait for other bots to do their first cycle
                stopShooting(),
                deployAndIntake(),
                path.cmd(),
                shootCommand().alongWith(startAgitating())
            )
        );

        return routine;
    }

    /*
     * Two cycle auto that goes bump then trench
     */
    public AutoRoutine tb(AutoPosition position) {
        String pathName = switch (position) {
            case LEFT -> "LeftTB";
            case RIGHT -> "RightTB";
            default -> "";
        };

        if (pathName.isBlank())
            return nothingAuto;

        AutoRoutine routine = autoFactory.newRoutine("routine");

        AutoTrajectory path = routine.trajectory(pathName, 0);
        path.atTime("Intake").onTrue(deployAndIntake());
        path.atTime("Spinup").onTrue(spinupShooter());
        path.atTime("Shoot").onTrue(shootCommand());
        
        AutoTrajectory path2 = routine.trajectory(pathName, 1);
        path2.atTime("Intake2").onTrue(deployAndIntake());
        path2.atTime("Spinup2").onTrue(spinupShooter());
        path2.atTime("Shoot2").onTrue(shootCommand().alongWith(startAgitating()));

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                path.cmd()
            )
        );

        path.done().onTrue(
            new SequentialCommandGroup(
                new WaitCommand(4), //Shooting time after first cycle
                stopShooting(),
                path2.cmd()
            )
        );

        return routine;
    }

    /*
     * Two cycle auto that goes bump then trench
     */
    public AutoRoutine tt(AutoPosition position) {
        String pathName = switch (position) {
            case LEFT -> "LeftTT";
            case RIGHT -> "RightTT";
            default -> "";
        };

        if (pathName.isBlank())
            return nothingAuto;

        AutoRoutine routine = autoFactory.newRoutine("routine");

        AutoTrajectory path = routine.trajectory(pathName, 0);
        path.atTime("Intake").onTrue(deployAndIntake());
        path.atTime("Spinup").onTrue(spinupShooter());
        path.atTime("Shoot").onTrue(shootCommand());
        
        AutoTrajectory path2 = routine.trajectory(pathName, 1);
        path2.atTime("Intake2").onTrue(deployAndIntake());
        path2.atTime("Spinup2").onTrue(spinupShooter());
        path2.atTime("Shoot2").onTrue(shootCommand().alongWith(startAgitating()));

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                path.cmd()
            )
        );

        path.done().onTrue(
            new SequentialCommandGroup(
                new WaitCommand(4), //Shooting time after first cycle
                stopShooting(),
                path2.cmd()
            )
        );

        return routine;
    }
}