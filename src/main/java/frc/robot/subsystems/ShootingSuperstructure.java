package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShotTrajectoryCalculator;

public class ShootingSuperstructure extends SubsystemBase {
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;

    private final CANrange fuelSensor;
    private final CANrangeConfiguration fuelSensorConfig = new CANrangeConfiguration();

    private final String turretLimelight = "turret_limelight";

    private Translation3d hubTarget = null;

    //TODO: Find actual height for targets
    private final Translation3d BLUE_HUB_TARGET = new Translation3d();
    private final Translation3d RED_HUB_TARGET = new Translation3d();

    //TODO: Tune optimal/most consistant value
    private final double HUB_TRAJECTORY_MAX_HEIGHT_FEET = 13;

    //TODO: Find actual offset from robot
    private final Transform3d TURRET_TRANSFORM = new Transform3d(0, 0, 0, Rotation3d.kZero);

    //TODO: Find correct CAN ID
    private final int CAN_RANGE_ID = 0;

    public ShootingSuperstructure() {
        shooter = new ShooterSubsystem();
        turret = new TurretSubsystem();

        fuelSensor = new CANrange(CAN_RANGE_ID);

        //TODO: Configure CANrange sensor
        fuelSensor.getConfigurator().apply(fuelSensorConfig);
        
        //Retrieve the correct hub position to target
        hubTarget = (DriverStation.getAlliance().get().equals(Alliance.Blue)) ? BLUE_HUB_TARGET : RED_HUB_TARGET;
    }

    /**
     * Reset the turret, hood, and flywheel back to their homed states (zeroed and unpowered)
     */
    public Command home() {
        return new SequentialCommandGroup(
            shooter.homeSubsystem(),
            turret.homeSubsystem()
        );
    }

    /**
     * Adjusts the turret, hood, and flywheel to compensate for robot movement to aim into the hub
     */
    public Command trackHub(Supplier<Pose3d> robotPose, Supplier<ChassisSpeeds> robotVelocity) {
        return run(() -> {
            ShotTrajectoryCalculator.update(
                robotPose.get().transformBy(TURRET_TRANSFORM),
                robotVelocity.get(),
                hubTarget,
                HUB_TRAJECTORY_MAX_HEIGHT_FEET
            );

            shooter.spinToRPM(() -> ShotTrajectoryCalculator.getTargetFlywheelRPM());
            shooter.setHoodPosition(() -> Units.degreesToRotations(ShotTrajectoryCalculator.getHoodAngle()));
            turret.rotateToAngle(() -> ShotTrajectoryCalculator.getTurretAngle());
        });
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
