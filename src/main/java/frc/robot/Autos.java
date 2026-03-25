package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSuperstructure.ShooterState;

public class Autos {
    private final AutoFactory autoFactory;

    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;

    public Autos(AutoFactory autoFactory, IntakeSubsystem intake, ShootingSuperstructure shooter) {
        this.autoFactory = autoFactory;
        
        this.intake = intake;
        this.shooter = shooter;
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
    
    public AutoRoutine rightBumpAuto() {
        String pathName = "RightBumpAuto";
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
                new WaitCommand(5), //Shooting time after first cycle
                stopAgitating(),
                stopShooting(),
                secondCycle.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine leftBumpAuto() {
        String pathName = "LeftBumpAuto";
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
                new WaitCommand(5), //Shooting time after first cycle
                stopAgitating(),
                stopShooting(),
                secondCycle.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine rightOPAuto() {
        String pathName = "RightOPAuto";
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

    public AutoRoutine leftOPAuto() {
        String pathName = "LeftOPAuto";
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

    // public AutoRoutine left2Cycle() {
    //     String pathName = "L2Cycle";

    //     AutoRoutine routine = autoFactory.newRoutine("routine");

    //     AutoTrajectory path = routine.trajectory(pathName, 0);
    //     path.atTime("StartIntaking").onTrue(deployAndIntake());
    //     path.atTime("StopIntaking").onTrue(intake.stopCommand());
    //     path.atTime("SpinUp").onTrue(shooter.spinUp(2000));
    //     path.atTime("StartShooting").onTrue(shooter.shoot());

    //     AutoTrajectory path2 = routine.trajectory(pathName, 1);
    //     path2.atTime("StartIntaking2").onTrue(deployAndIntake());
    //     path2.atTime("StopIntaking2").onTrue(intake.stopCommand());
    //     path2.atTime("SpinUp2").onTrue(shooter.spinUp(2000));
    //     path2.atTime("StartShooting2").onTrue(shooter.shoot().alongWith(startAgitating()));

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
}