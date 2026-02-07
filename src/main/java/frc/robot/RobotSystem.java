package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.TransferSubsystem;

public class RobotSystem extends SubsystemBase {
    private final IntakeSubsystem intake;
    private final TransferSubsystem transfer;
    private final ShootingSuperstructure shooter;

    public RobotSystem() {
        intake = new IntakeSubsystem();
        transfer = new TransferSubsystem();
        shooter = new ShootingSuperstructure();
    }

    @Override
    public void periodic() {
        
    }
}
