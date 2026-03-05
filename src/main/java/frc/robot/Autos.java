package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    /*
     * Goes to center and does one sweep and then comes back and shoots on the move 
     * and ends up in climb position.
     */
    public AutoRoutine oneCycle() {
        AutoRoutine routine = autoFactory.newRoutine("routine");

        AutoTrajectory start = routine.trajectory("ShootingAuto");
        AutoTrajectory goToCenter = routine.trajectory("ShootingAuto", 0);
        AutoTrajectory goBackToShoot = routine.trajectory("ShootingAuto", 1);
        AutoTrajectory goClimb = routine.trajectory("ShootingAuto", 2);

        routine.active().onTrue(
            new SequentialCommandGroup(
                start.resetOdometry(),
                shooter.autonomousShoot(),
                start.cmd(),
                goToCenter.cmd(), //Along with deploying intake
                goBackToShoot.cmd(), //Along with retracting intake
                shooter.autonomousShoot(),
                goClimb.cmd()
            )
        );

        return routine;
    }
}
