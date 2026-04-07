package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
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
        return shooter.spinUp(2800);
    }

    private Command startShooting() {
        return shooter.shoot().alongWith(
            new SequentialCommandGroup(
                intake.agitate(() -> 0.5), //One quick agitate to start the ball rolling (pun intended)
                new WaitCommand(1.5),
                intake.agitate(() -> 0.75).repeatedly()
            )
        );
    }

    private Command stopShooting() {
        return intake.stopCommand();
    }

    public AutoRoutine testAuto() {
        AutoRoutine routine = autoFactory.newRoutine("routine");
        AutoTrajectory path = routine.trajectory("TestAuto");

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                new ScheduleCommand(deployAndIntake()),
                path.cmd()
            )
        );

        path.done().onTrue(
            new SequentialCommandGroup(
                new ScheduleCommand(startShooting()),
                new WaitCommand(5),
                new ScheduleCommand(stopShooting()),
                new ScheduleCommand(retractAndStopIntake())
            )
        );

        return routine;
    }

    public AutoRoutine leftBear() {
        String pathName = "LeftBear";

        AutoRoutine routine = autoFactory.newRoutine("routine");

        AutoTrajectory path = routine.trajectory(pathName, 0);
        path.atTime("Intake").onTrue(deployAndIntake());
        path.atTime("Spinup").onTrue(spinupShooter());
        path.atTime("Shoot").onTrue(startShooting());
        path.atTime("Depot").onTrue(stopShooting().andThen(deployAndIntake()));

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                new WaitCommand(4),
                path.cmd()
            )
        );

        path.done().onTrue(
            startShooting()
        );

        return routine;
    }

    /*
     * Two cycle auto that goes bump then trench
     */
    public AutoRoutine bumpTrench(AutoStartingPosition position) {
        String pathName = switch (position) {
            case LEFT -> "LeftBT";
            case RIGHT -> "RightBT";
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
                new WaitCommand(4), //Shooting time after first cycle
                stopShooting(),
                path2.cmd()
            )
        );

        return routine;
    }

    /*
     * Two cycle auto that goes through the trench twice
     */
    public AutoRoutine doubleTrench(AutoStartingPosition position) {
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
}