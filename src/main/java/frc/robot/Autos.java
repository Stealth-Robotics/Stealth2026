package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSuperstructure.ShooterState;

public class Autos {
    private final AutoFactory autoFactory;

    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;
    private final ClimbSubsystem climb;

    public Autos(AutoFactory autoFactory, DriveSubsystem drive, IntakeSubsystem intake, ShootingSuperstructure shooter, ClimbSubsystem climb) {
        this.autoFactory = autoFactory;
        
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.climb = climb;
    }

    public AutoRoutine rightOneCyclePlusOutpost() {
        AutoRoutine routine = autoFactory.newRoutine("routine");

        String pathName = "RightOneCyclePlusOutpost";

        AutoTrajectory path = routine.trajectory(pathName, 0);
        path.atTime("StartIntaking").onTrue(intake.startIntaking());
        path.atTime("SpinUp").onTrue(shooter.spinUp(2500));
        path.atTime("StartShooting").onTrue(shooter.shoot().alongWith(intake.startAgitate()));
        path.atTime("StopShooting").onTrue(shooter.shoot());

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                new InstantCommand(() -> shooter.setState(ShooterState.HUB_TRACKING)),
                path.cmd()
            )
        );
        path.done().onTrue(
            new SequentialCommandGroup(
                intake.stopAgitate(),
                new InstantCommand(() -> shooter.setState(ShooterState.IDLE))
            )
        );

        return routine;
    }
    public AutoRoutine leftOneCyclePlusDepot() {
        AutoRoutine routine = autoFactory.newRoutine("routine");

        String pathName = "LeftOneCyclePlusDepotSlow";

        AutoTrajectory path = routine.trajectory(pathName,0);
        path.atTime("StartIntaking").onTrue(intake.startIntaking());
        path.atTime("StopIntaking").onTrue(intake.stopIntaking());
        path.atTime("SpinUp").onTrue(shooter.spinUp(2500));
        path.atTime("StartShooting").onTrue(shooter.shoot().alongWith(intake.startAgitate()));
        path.atTime("StopShooting").onTrue(intake.startIntaking().alongWith(shooter.shoot()));

        AutoTrajectory path2 = routine.trajectory(pathName,1);

        routine.active().onTrue(
            new SequentialCommandGroup(
                path.resetOdometry(),
                new InstantCommand(() -> shooter.setState(ShooterState.HUB_TRACKING)),
                path.cmd()
            )
        );

        path.done().onTrue(
            new SequentialCommandGroup(
                new WaitCommand(2),
                intake.startAgitate(),
                new ParallelDeadlineGroup(
                    path2.cmd(),
                    shooter.shoot()
                ),
                intake.stopAgitate(),
                new InstantCommand(() -> shooter.setState(ShooterState.IDLE))
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
                new InstantCommand(() -> shooter.setState(ShooterState.HUB_TRACKING)),
                intake.startAgitate(),
                shooter.shoot(),
                path.cmd()
            )
        );

        path.done().onTrue(
            new SequentialCommandGroup(
                intake.stopAgitate(),
                new InstantCommand(() -> shooter.setState(ShooterState.IDLE))
            )
        );

        return routine;
    }
    // public AutoRoutine rightCenterAutoClimb() {
    //     AutoRoutine routine = autoFactory.newRoutine("rightCenterAutoClimb");
    //     String pathName = "RightCenterClimb";
        
    //     AutoTrajectory driveWhileShooting = routine.trajectory(pathName,0);
    //     AutoTrajectory climbAlign = routine.trajectory(pathName,1);

    //     routine.active().onTrue(
    //         new SequentialCommandGroup(
    //             driveWhileShooting.resetOdometry(),
    //             new InstantCommand(() -> shooter.setState(ShooterState.HUB_TRACKING)),
    //             shooter.shoot(),
    //             driveWhileShooting.cmd(),
    //             new WaitCommand(0.5),
    //             new InstantCommand(() -> shooter.setState(ShooterState.IDLE)),
    //             climbAlign.cmd()
    //         )
    //     );
    //     return routine;
    // }
    //     public AutoRoutine leftCenterAutoClimb() {
    //     AutoRoutine routine = autoFactory.newRoutine("leftCenterAutoClimb");
    //     String pathName = "LeftCenterClimb";
        
    //     AutoTrajectory driveWhileShooting = routine.trajectory(pathName,0);
    //     AutoTrajectory climbAlign = routine.trajectory(pathName,1);

    //     routine.active().onTrue(
    //         new SequentialCommandGroup(
    //             driveWhileShooting.resetOdometry(),
    //             new InstantCommand(() -> shooter.setState(ShooterState.HUB_TRACKING)),
    //             shooter.shoot(),
    //             driveWhileShooting.cmd(),
    //             new WaitCommand(0.5),
    //             new InstantCommand(() -> shooter.setState(ShooterState.IDLE)),
    //             climbAlign.cmd()
    //         )
    //     );
    //     return routine;
    // }
}