package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.TransferSubsystem;

@SuppressWarnings("unused")
public class RobotSystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final TransferSubsystem transfer;
    private final ShootingSuperstructure shooter;
    private final ClimbSubsystem climb;

    private final Command driverRumble, operatorRumble;


    public RobotSystem(Command driverRumble, Command operatorRumble) {
        drive = TunerConstants.createDrivetrain();
        intake = new IntakeSubsystem();
        transfer = new TransferSubsystem();
        shooter = new ShootingSuperstructure();
        climb = new ClimbSubsystem();

        this.driverRumble = driverRumble;
        this.operatorRumble = operatorRumble;
    }

    /**
     * @param x The supplier for driving the robot forward (field centric)
     * @param y The supplier for driving the robot sideways (field centric)
     * @param theta The supplier for rotating the robot
     * @param isFieldCentric Supplier that allows us to toggle between driving modes
     */
    public void setDriveDefaultCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, BooleanSupplier isFieldCentric) {
        drive.setDefaultCommand(
            new ConditionalCommand(
                drive.applyRequest(() -> drive.fieldCentric
                    .withVelocityX(y.getAsDouble() * drive.MAX_SPEED)
                    .withVelocityY(x.getAsDouble() * drive.MAX_SPEED)
                    .withRotationalRate(theta.getAsDouble() * drive.MAX_ANGULAR_RATE)
                ),
                drive.applyRequest(() -> drive.robotCentric
                        .withVelocityX(-y.getAsDouble() * drive.MAX_SPEED)
                        .withVelocityY(-x.getAsDouble() * drive.MAX_SPEED)
                        .withRotationalRate(theta.getAsDouble() * drive.MAX_ANGULAR_RATE)
                ),
                isFieldCentric
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

    public Autos getAutos() {
        return new Autos(
            drive.createAutoFactory(),
            drive,
            intake,
            transfer,
            shooter,
            climb
        );
    }

    @Override
    public void periodic() {    
    }
}
