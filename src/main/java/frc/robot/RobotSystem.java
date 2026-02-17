package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.util.ShiftTracker;

public class RobotSystem extends SubsystemBase {
    private final IntakeSubsystem intake;
    private final TransferSubsystem transfer;
    private final ShootingSuperstructure shooter;
    private final ClimbSubsystem climb;
    private final DriveSubsystem drive;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //private final Telemetry logger = new Telemetry(MaxSpeed);

    public RobotSystem() {
        intake = new IntakeSubsystem();
        transfer = new TransferSubsystem();
        shooter = new ShootingSuperstructure();
        climb = new ClimbSubsystem();
        drive = TunerConstants.createDrivetrain();
        configureBindings();
        
    }

    private void configureBindings() {
           drive.setDefaultCommand(
            drive.applyRequest(() ->
                fieldCentric.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
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
