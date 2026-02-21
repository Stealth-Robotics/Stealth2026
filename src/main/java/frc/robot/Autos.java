package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.TransferSubsystem;

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

    public AutoRoutine nothingAuto() {
        AutoRoutine routine = autoFactory.newRoutine("NothingAuto");
        return routine;
    }
}
