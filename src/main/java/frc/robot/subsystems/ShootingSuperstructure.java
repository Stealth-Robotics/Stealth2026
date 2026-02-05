package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShootingSuperstructure extends SubsystemBase {
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;

    public ShootingSuperstructure() {
        shooter = new ShooterSubsystem();
        turret = new TurretSubsystem();
    }

    /**
     * Reset the turret, hood, and flywheel back to their homed states (zeroed and unpowered)
     */
    public Command home() {
        return new SequentialCommandGroup(
            shooter.homeSubsystem()
        );
    }

    /**
     * Adjusts the turret, hood, and flywheel to compensate for robot movement to aim into the hub
     */
    public Command trackHub() {
        return new SequentialCommandGroup(
            
        );
    }

    /**
     * Adjusts the turret, hood, and flywheel to shoot into our alliance area
     */
    public Command pass() {
        return new SequentialCommandGroup(
            
        );
    }

    @Override
    public void periodic() {

    }
}
