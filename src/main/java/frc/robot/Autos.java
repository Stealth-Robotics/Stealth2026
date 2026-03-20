package frc.robot;

import java.util.function.DoubleSupplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    private Command autonomousShoot(DoubleSupplier seconds) {
        return new ParallelDeadlineGroup(
            new WaitCommand(seconds.getAsDouble()),
            shooter.shoot().alongWith(intake.agitate().repeatedly())
        ).finallyDo(() -> {
            shooter.shoot().cancel();
            intake.agitate().cancel();
            shooter.idleSubsystems();
        });
    }

    public AutoRoutine rightOneCyclePlusOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("routine");

        String pathName = "RightOneCyclePlusOutpost";

        AutoTrajectory path = routine.trajectory(pathName, 0);
        path.atTime("StartIntaking").onTrue(intake.intakeCommand().andThen(intake.deployCommand()));
        path.atTime("SpinUp").onTrue(shooter.spinUp(2500).alongWith(intake.stopCommand()));
        path.atTime("StartShooting").onTrue(shooter.shoot());
        path.atTime("StopShooting").onTrue(shooter.shoot());

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                path.cmd()
            )
        );
        path.done().onTrue(
            new SequentialCommandGroup(
                new WaitCommand(5),
                shooter.shoot().alongWith(intake.agitate().repeatedly()).alongWith(intake.intakeCommand())
            )
        );

        return routine;
    }

    public AutoRoutine leftOneCyclePlusDepot() {
        AutoRoutine routine = autoFactory.newRoutine("routine");

        String pathName = "LeftOneCyclePlusDepotSlow";

        AutoTrajectory path = routine.trajectory(pathName, 0);
        path.atTime("StartIntaking").onTrue(intake.intakeCommand().andThen(intake.deployCommand()));
        path.atTime("StopIntaking").onTrue(intake.stopCommand());
        path.atTime("SpinUp").onTrue(shooter.spinUp(2500));
        path.atTime("StartShooting").onTrue(autonomousShoot(() -> 2));

        AutoTrajectory path2 = routine.trajectory(pathName, 1);
        path2.atTime("StartIntaking2").onTrue(intake.intakeCommand().andThen(intake.deployCommand()));
        path2.atTime("StopIntaking2").onTrue(intake.stopCommand());
        path2.atTime("SpinUp2").onTrue(shooter.spinUp(2500));
        path2.atTime("StartShooting2").onTrue(autonomousShoot(() -> 10));

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                path.cmd()
            )
        );

        path.done().onTrue(
            new SequentialCommandGroup(
                new WaitCommand(2),
                intake.retractCommand(),
                path2.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine centerPreload() {
        AutoRoutine routine = autoFactory.newRoutine("routine");

        String pathName = "CenterPreload";

        AutoTrajectory path = routine.trajectory(pathName, 0);

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                shooter.shoot(),
                path.cmd()
            )
        );

        return routine;
    }
    
    public AutoRoutine simpleCenterAuto() {
        AutoRoutine routine = autoFactory.newRoutine("simpleCenterAuto");
        String pathName = "SimpleShoot";
        
        AutoTrajectory drive = routine.trajectory(pathName,0);
        AutoTrajectory shoot = routine.trajectory(pathName,1);
        AutoTrajectory stopspot = routine.trajectory(pathName,2);

        routine.active().onTrue(
            Commands.sequence(
                drive.resetOdometry(),
                new InstantCommand(() -> shooter.setState(ShooterState.HUB_TRACKING)),
                drive.cmd().withTimeout(4))
        );

        drive.done().onTrue(
            Commands.sequence(
                shooter.shoot(),
                new WaitCommand(5)
            ).andThen(shoot.cmd()).withTimeout(4)
        );

        shoot.done().onTrue(stopspot.cmd());
        
        return routine;
    }

    //Irrelevent until we have a climb
    // public AutoRoutine centerClimbAuto() {
    //     AutoRoutine routine = autoFactory.newRoutine("centerClimbAuto");
    //     String pathName = "CenterPreloadPlusClimb";

    //     AutoTrajectory path = routine.trajectory(pathName, 0);

    //     path.atTime("DeployClimb").onTrue(climb.reach());
    //     path.atTime("StartShoot").onTrue(shooter.shoot());

    //     routine.active().onTrue(
    //         Commands.sequence(
    //             path.resetOdometry(),
    //             new InstantCommand(() -> shooter.setState(ShooterState.HUB_TRACKING)),
    //             shooter.spinUp(1500),
    //             path.cmd().withTimeout(18)
    //         )
    //     );
    //     path.done().onTrue(
    //         climb.ascend()
    //     );
    //     return routine;
    // }
}