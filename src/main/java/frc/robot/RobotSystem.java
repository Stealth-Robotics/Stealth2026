package frc.robot;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.TransferSubsystem;

public class RobotSystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final TransferSubsystem transfer;
    private final ShootingSuperstructure shooter;
    private final ClimbSubsystem climb;

    public RobotSystem() {
        drive = TunerConstants.createDrivetrain();
        intake = new IntakeSubsystem();
        transfer = new TransferSubsystem();
        shooter = new ShootingSuperstructure();
        climb = new ClimbSubsystem();
    }

    /**
     * @param x The supplier for driving the robot forward (field centric)
     * @param y The supplier for driving the robot sideways (field centric)
     * @param theta The supplier for rotating the robot
     */
    public void setDriveDefaultCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        drive.setDefaultCommand(
            drive.applyRequest(() -> drive.fieldCentric
                .withVelocityX(y.getAsDouble() * drive.MAX_SPEED)
                .withVelocityY(x.getAsDouble() * drive.MAX_SPEED)
                .withRotationalRate(theta.getAsDouble() * drive.MAX_ANGULAR_RATE)
            )
        );
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
