package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.TransferSubsystem;

public class RobotSystem extends SubsystemBase {
    private final IntakeSubsystem intake;
    private final TransferSubsystem transfer;
    private final ShootingSuperstructure shooter;
    private final ClimbSubsystem climb;

    public RobotSystem() {
        intake = new IntakeSubsystem();
        transfer = new TransferSubsystem();
        shooter = new ShootingSuperstructure();
        climb = new ClimbSubsystem();
    }

     /**
     * Only allows us to shoot if the turret is tracking (not wrapping or at limit), the shooter is at speed, and
     * we are not driving/spinning to quickly
     */
    public boolean isShootingValid() {
        return shooter.isShootingValid();
    }

    @Override
    public void periodic() {
        
    }
}
