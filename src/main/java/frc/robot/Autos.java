package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.AutoStartingPosition;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;

public class Autos {
    private final AutoFactory autoFactory;
    private final AutoRoutine nothingAuto;

    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;

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
        autoFactory.cache().loadTrajectory(autoName);
    }

    private Command stopAgitating() {
        return new InstantCommand(() -> intake.stopCommand());
    }

    private Command deployAndIntake() {
        return intake.deployCommand().andThen(intake.intakeCommand());
    }

    private Command retractAndStopIntake() {
        return intake.retractCommand().andThen(intake.stopCommand());
    }

    private Command spinupShooter() {
        return shooter.spinUp(2500);
    }

    private Command startShooting() {
        return shooter.shoot().alongWith(
            new SequentialCommandGroup(
                new WaitCommand(1.5),
                intake.agitate().repeatedly()
            )
        );
    }

    private Command stopShooting() {
        return shooter.stopShooting().andThen(stopAgitating());
    }

    /*
     * Delayed middle steal auto
     */
    // public AutoRoutine middle(AutoStartingPosition position) {
    //     String pathName = switch (position) {
    //         case LEFT -> "LeftMiddle";
    //         case RIGHT -> "RightMiddle";
    //         default -> "";
    //     };

    //     if (pathName.isBlank())
    //         return nothingAuto;

    //     AutoRoutine routine = autoFactory.newRoutine("routine");

    //     AutoTrajectory path = routine.trajectory(pathName, 0);

    //     routine.active().onTrue(
    //         new SequentialCommandGroup(
    //             path.resetOdometry(),
    //             shootCommand(),
    //             new WaitCommand(5), //Wait for other bots to do their first cycle
    //             stopShooting(),
    //             deployAndIntake(),
    //             path.cmd(),
    //             shootCommand().alongWith(startAgitating())
    //         )
    //     );

    //     return routine;
    // }

    /*
     * Two cycle auto that goes bump then trench
     */
    // public AutoRoutine tb(AutoStartingPosition position) {
    //     String pathName = switch (position) {
    //         case LEFT -> "LeftTB";
    //         case RIGHT -> "RightTB";
    //         default -> "";
    //     };

    //     if (pathName.isBlank())
    //         return nothingAuto;

    //     AutoRoutine routine = autoFactory.newRoutine("routine");

    //     AutoTrajectory path = routine.trajectory(pathName, 0);
    //     path.atTime("Intake").onTrue(deployAndIntake());
    //     path.atTime("Spinup").onTrue(spinupShooter());
    //     path.atTime("Shoot").onTrue(shootCommand());
        
    //     AutoTrajectory path2 = routine.trajectory(pathName, 1);
    //     path2.atTime("Intake2").onTrue(deployAndIntake());
    //     path2.atTime("Spinup2").onTrue(spinupShooter());
    //     path2.atTime("Shoot2").onTrue(shootCommand().alongWith(startAgitating()));

    //     routine.active().onTrue(
    //         new SequentialCommandGroup(
    //             path.resetOdometry(),
    //             path.cmd()
    //         )
    //     );

    //     path.done().onTrue(
    //         new SequentialCommandGroup(
    //             new WaitCommand(4), //Shooting time after first cycle
    //             stopShooting(),
    //             path2.cmd()
    //         )
    //     );

    //     return routine;
    // }

    public AutoRoutine testAuto() {
        AutoRoutine routine = autoFactory.newRoutine("routine");

        routine.active().onTrue(
            new SequentialCommandGroup(
                deployAndIntake(),
                new WaitCommand(2),
                spinupShooter(),
                new WaitCommand(2),
                startShooting().withTimeout(5),
                stopShooting(),
                retractAndStopIntake()
            )
        );

        return routine;
    }

    /*
     * Two cycle auto that goes bump then trench
     */
    public AutoRoutine tt(AutoStartingPosition position) {
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
        path.atTime("Shoot").onTrue(startShooting());
        
        AutoTrajectory path2 = routine.trajectory(pathName, 1);
        path2.atTime("Intake2").onTrue(deployAndIntake());
        path2.atTime("Spinup2").onTrue(spinupShooter());
        path2.atTime("Shoot2").onTrue(startShooting());

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                path.cmd()
            )
        );

        path.done().onTrue(
            new SequentialCommandGroup(
                new WaitCommand(5), //Shooting time after first cycle
                stopShooting(),
                path2.cmd()
            )
        );

        return routine;
    }
    public AutoRoutine bearAuto(AutoStartingPosition position) {
        String pathName = switch (position) {
            case LEFT -> "LeftBear";
            case RIGHT -> "RightBear";
            default -> "";
        };
        if (pathName.isBlank())
            return nothingAuto;
        
        AutoRoutine routine = autoFactory.newRoutine("routine");
        AutoTrajectory path = routine.trajectory(pathName, 0);
        path.atTime("Intake").onTrue(deployAndIntake());
        path.atTime("Spinup").onTrue(spinupShooter());
        path.atTime("Shoot").onTrue(shooter.shoot().alongWith(startAgitating()));

        AutoTrajectory path2 = routine.trajectory(pathName, 1);

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                new WaitCommand(5),
                path.cmd()
            )
        );

        path.done().onTrue(
            new SequentialCommandGroup(
                new WaitCommand(4), //Shooting time after first cycle
                path2.cmd()
            )
        );

        return routine;    
    }
}