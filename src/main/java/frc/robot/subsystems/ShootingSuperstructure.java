package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShootingSuperstructure extends SubsystemBase {
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;

    private final CANrange fuelSensor;

    private final String turretLimelight = "turret_limelight";

    //TODO: Find actual values from robot
    private final double TURRET_OFFSET_X_INCHES = 0;
    private final double TURRET_OFFSET_Y_INCHES = 0;

    //TODO: Find correct CAN ID
    private final int CAN_RANGE_ID = 0;

    public ShootingSuperstructure() {
        shooter = new ShooterSubsystem();
        turret = new TurretSubsystem();

        fuelSensor = new CANrange(CAN_RANGE_ID);
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
