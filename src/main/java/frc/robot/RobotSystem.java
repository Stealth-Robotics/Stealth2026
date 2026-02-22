package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.ShootingSuperstructure.ShooterState;
import frc.robot.util.ShiftTracker;
import frc.robot.util.ZoneManager;

public class RobotSystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;
    private final ClimbSubsystem climb;

    private final Command driverRumble, operatorRumble;

    public RobotSystem(Command driverRumble, Command operatorRumble) {
        drive = TunerConstants.createDrivetrain();
        intake = new IntakeSubsystem();
        shooter = new ShootingSuperstructure(() -> drive.getPose(), () -> drive.getFieldRelativeVelocity());
        climb = new ClimbSubsystem();

        this.driverRumble = driverRumble;
        this.operatorRumble = operatorRumble;
    }

    public Command shoot() {
        return shooter.shoot()
        .onlyIf(() -> ShiftTracker.canScore() || SmartDashboard.getBoolean("Force Allow Shooting", false));
    }

    private void updateShootingState() {
        if (ZoneManager.inHubZone())
            shooter.setState(ShooterState.HUB_TRACKING);
        else if (ZoneManager.inPassingZone())
            shooter.setState(ShooterState.PASSING);
        else
            shooter.setState(ShooterState.IDLE);
    }

    /**
     * @param x The supplier for driving the robot forward (field centric)
     * @param y The supplier for driving the robot sideways (field centric)
     * @param theta The supplier for rotating the robot
     * @param isFieldCentric Supplier that allows us to toggle between driving modes
     */
    public void setDriveDefaultCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, BooleanSupplier isFieldCentric) {
        drive.setDefaultCommand(
            drive.applyRequest(() -> {
                return isFieldCentric.getAsBoolean() ? 
                    drive.fieldCentric
                        .withVelocityX(y.getAsDouble() * drive.MAX_SPEED)
                        .withVelocityY(x.getAsDouble() * drive.MAX_SPEED)
                        .withRotationalRate(theta.getAsDouble() * drive.MAX_ANGULAR_RATE) :
                    drive.robotCentric
                        .withVelocityX(-y.getAsDouble() * drive.MAX_SPEED)
                        .withVelocityY(-x.getAsDouble() * drive.MAX_SPEED)
                        .withRotationalRate(theta.getAsDouble() * drive.MAX_ANGULAR_RATE);
            })
        );
    }

    public Command resetRobotHeading() {
        return runOnce(() -> drive.seedFieldCentric());
    }

    public Autos getAutos() {
        return new Autos(
            drive.createAutoFactory(),
            drive,
            intake,
            shooter,
            climb
        );
    }

    @Override
    public void periodic() {
        ZoneManager.updateWithRobotPose(drive.getPose());
        updateShootingState();
    }
}
