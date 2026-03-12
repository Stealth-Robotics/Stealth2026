package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;

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

        AutoTrajectory goToCenter = routine.trajectory(pathName, 0);
        AutoTrajectory startIntaking = routine.trajectory(pathName, 1);
        AutoTrajectory stopIntaking = routine.trajectory(pathName, 2);
        AutoTrajectory startShooting = routine.trajectory(pathName, 3);
        AutoTrajectory goToOutpost = routine.trajectory(pathName, 4);

        routine.active().onTrue(
            new SequentialCommandGroup(
                goToCenter.resetOdometry(),
                goToCenter.cmd(),

                intake.deployCommand(),
                intake.intakeCommand(),

                startIntaking.cmd(),
                stopIntaking.cmd(),

                intake.stopCommand(),

                new ParallelDeadlineGroup(
                    startShooting.cmd(),
                    shooter.shoot()
                ),

                goToOutpost.cmd(),
                new WaitCommand(2),
                drive.applyRequest(() -> drive.brake),
                shooter.shoot()
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
    //             shooter.shoot(),
    //             driveWhileShooting.resetOdometry(),
    //             driveWhileShooting.cmd(),
    //             shooter.
                
    //         )
    //     );
    // }
}