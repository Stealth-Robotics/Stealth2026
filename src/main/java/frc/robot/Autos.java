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
import frc.robot.util.AutoStartingPosition;

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

    private Command stopShooting() {
        return new InstantCommand(() -> shooter.shoot().cancel());
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
        return intake.deployCommand().andThen(intake.intakeCommand());
    }

    private Command spinupShooter() {
        return shooter.spinUp(2500);
    }

    public AutoRoutine middleDepot() {
        String pathName = "MiddleDepot";
        AutoRoutine routine = autoFactory.newRoutine("routine");

        AutoTrajectory firstCycle = routine.trajectory(pathName, 0);
        firstCycle.atTime("Intake").onTrue(deployAndIntake());
        firstCycle.atTime("End").onTrue(shooter.shoot().alongWith(startAgitating()));

        routine.active().onTrue(
            new SequentialCommandGroup(
                firstCycle.resetOdometry(),
                firstCycle.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine doubleBump(AutoPosition position) {
        String pathName = switch (position) {
            case LEFT -> "LeftBB";
            case RIGHT -> "RightBB";
            default -> "";
        };

        if (pathName.isBlank())
            return nothingAuto;

        AutoRoutine routine = autoFactory.newRoutine("routine");

        AutoTrajectory firstCycle = routine.trajectory(pathName, 0);
        firstCycle.atTime("Intake").onTrue(deployAndIntake());
        firstCycle.atTime("Spinup").onTrue(spinupShooter());
        firstCycle.atTime("Shoot").onTrue(shooter.shoot().alongWith(startAgitating()));

        AutoTrajectory secondCycle = routine.trajectory(pathName, 1);
        secondCycle.atTime("Intake2").onTrue(deployAndIntake());
        secondCycle.atTime("Spinup2").onTrue(spinupShooter());
        secondCycle.atTime("Shoot2").onTrue(shooter.shoot().alongWith(startAgitating()));

        routine.active().onTrue(
            new SequentialCommandGroup(
                firstCycle.resetOdometry(),
                firstCycle.cmd()
            )
        );

        firstCycle.done().onTrue(
            new SequentialCommandGroup(
                new WaitCommand(5), //Shooting time after first cycle
                deployAndIntake(),
                secondCycle.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine bumpTrench(AutoPosition position) {
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
        path.atTime("Shoot").onTrue(shooter.shoot().alongWith(startAgitating()));
        
        AutoTrajectory path2 = routine.trajectory(pathName, 1);
        path2.atTime("Intake2").onTrue(deployAndIntake());
        path2.atTime("Spinup2").onTrue(spinupShooter());
        path2.atTime("Shoot2").onTrue(shooter.shoot().alongWith(startAgitating()));

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                path.cmd()
            )
        );

        path.done().onTrue(
            new SequentialCommandGroup(
                new WaitCommand(4), //Shooting time after first cycle
                deployAndIntake(),
                path2.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine doubleTrench(AutoPosition position) {
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
        path.atTime("Shoot").onTrue(shooter.shoot().alongWith(startAgitating()));
        
        AutoTrajectory path2 = routine.trajectory(pathName, 1);
        path2.atTime("Intake2").onTrue(deployAndIntake());
        path2.atTime("Spinup2").onTrue(spinupShooter());
        path2.atTime("Shoot2").onTrue(shooter.shoot().alongWith(startAgitating()));

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                path.cmd()
            )
        );

        path.done().onTrue(
            new SequentialCommandGroup(
                new WaitCommand(5), //Shooting time after first cycle
                deployAndIntake(),
                path2.cmd()
            )
        );

        return routine;
    }
}