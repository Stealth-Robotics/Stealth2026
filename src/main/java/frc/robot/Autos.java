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
    public AutoRoutine sadMiddleBean(AutoPosition position) {
        String pathName = switch (position) {
            case LEFT -> "LeftSadBean";
            case RIGHT -> "RightSadBean";
            default -> "";
        };

        if (pathName.isBlank())
            return nothingAuto;

        AutoRoutine routine = autoFactory.newRoutine("routine");

        AutoTrajectory path = routine.trajectory(pathName, 0);

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                new WaitCommand(5), //Wait for other bots to do their first cycle
                deployAndIntake(),
                path.cmd(),
                shootCommand().alongWith(startAgitating())
            )
        );

        return routine;
    }

    /*
     * Goated auto that was definitely not stolen from 2056
     */
    public AutoRoutine doubleBean(AutoPosition position) {
        String pathName = switch (position) {
            case LEFT -> "LeftDoubleBean";
            case RIGHT -> "RightDoubleBean";
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

    public AutoRoutine doubleBump(AutoPosition position) {
        String pathName = switch (position) {
            case LEFT -> "LeftDoubleBump";
            case RIGHT -> "RightDoubleBump";
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
                new WaitCommand(6), //Shooting time after first cycle
                stopAgitating(),
                stopShooting(),
                secondCycle.cmd()
            )
        );

        return routine;
    }
    
    /*
     * Auto that does one sweep through the trench, shoots, and does another through the trench and back over the bump
     * to finish shooting.
     */
    public AutoRoutine trenchBump(AutoPosition position) {
        String pathName = switch (position) {
            case LEFT -> "LeftTrenchBump";
            case RIGHT -> "RightTrenchBump";
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
                stopAgitating(),
                stopShooting(),
                secondCycle.cmd()
            )
        );

        return routine;
    }

    /*
     * Auto that does two different sweeps through the trench and comes back to shoot.
     */
    public AutoRoutine trench2Cycle(AutoPosition position) {
        String pathName = switch (position) {
            case LEFT -> "LeftTrench2Cycle";
            case RIGHT -> "RightTrench2Cycle";
            default -> "";
        };

        if (pathName.isBlank())
            return nothingAuto;

        AutoRoutine routine = autoFactory.newRoutine("routine");

        AutoTrajectory firstCycle = routine.trajectory(pathName, 0);
        firstCycle.atTime("Intake").onTrue(deployAndIntake());
        firstCycle.atTime("Shoot").onTrue(shooter.shoot().alongWith(startAgitating()));

        AutoTrajectory secondCycle = routine.trajectory(pathName, 1);
        secondCycle.atTime("Intake2").onTrue(deployAndIntake());
        secondCycle.atTime("Shoot2").onTrue(shooter.shoot().alongWith(startAgitating()));

        routine.active().onTrue(
            new SequentialCommandGroup(
                firstCycle.resetOdometry(),
                firstCycle.cmd()
            )
        );

        firstCycle.done().onTrue(
            new SequentialCommandGroup(
                new WaitCommand(4), //Shooting time after first cycle
                stopAgitating(),
                stopShooting(),
                secondCycle.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine right1CyclePlusOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("routine");

        String pathName = "RightOneCyclePlusOutpost";

        AutoTrajectory sweepPath = routine.trajectory(pathName, 0);
        sweepPath.atTime("StartIntaking").onTrue(deployAndIntake());
        sweepPath.atTime("SpinUp").onTrue(shooter.spinUp(2500).andThen(intake.stopCommand()));
        sweepPath.atTime("StartShooting").onTrue(shooter.shoot().alongWith(startAgitating()));

        routine.active().onTrue(
            new SequentialCommandGroup(
                sweepPath.resetOdometry(),
                sweepPath.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine left1CyclePlusDepot() {
        AutoRoutine routine = autoFactory.newRoutine("routine");

        String pathName = "LeftOneCyclePlusDepot";

        AutoTrajectory cycle = routine.trajectory(pathName, 0);
        cycle.atTime("StartIntaking").onTrue(deployAndIntake());
        cycle.atTime("StopIntaking").onTrue(intake.stopCommand());
        cycle.atTime("SpinUp").onTrue(shooter.spinUp(2500));
        cycle.atTime("StartShooting").onTrue(shooter.shoot());
        cycle.atTime("StopShooting").onTrue(stopShooting());

        AutoTrajectory depot = routine.trajectory(pathName, 1);
        depot.atTime("Shoot2").onTrue(shooter.shoot().alongWith(startAgitating()));

        routine.active().onTrue(
            new SequentialCommandGroup(
                cycle.resetOdometry(),
                cycle.cmd()
            )
        );

        cycle.done().onTrue(
            new SequentialCommandGroup(
                new WaitCommand(1),
                depot.cmd()
            )
        );

        return routine;
    }
}