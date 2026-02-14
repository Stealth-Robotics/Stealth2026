package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import frc.robot.util.FieldConstants;
import frc.robot.util.ShotTrajectoryCalculator;

public class ShootingSuperstructure extends SubsystemBase {
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;

    private final double MAX_SHOT_SPACING_SECONDS = 0.5;

    private final CANrange shotSensor;
    private final int CAN_RANGE_ID = 0;
    private final CANrangeConfiguration shotSensorConfig = new CANrangeConfiguration();
    private final Debouncer shotDebouncer = new Debouncer(MAX_SHOT_SPACING_SECONDS, DebounceType.kFalling);

    private final String turretLimelight = "turret_limelight";

    // Aiming targets for the hub locations
    private final Translation3d hubTarget;

    public static final Translation3d BLUE_HUB = new Translation3d();
    public static final Translation3d RED_HUB = new Translation3d();

    // Aiming targets for the different passing locations
    private final Translation3d passLeftTarget;
    private final Translation3d passRightTarget;

    public static final Translation3d PASS_BLUE_LEFT = new Translation3d();
    public static final Translation3d PASS_BLUE_RIGHT = new Translation3d();

    public static final Translation3d PASS_RED_LEFT = new Translation3d();
    public static final Translation3d PASS_RED_RIGHT = new Translation3d();

    private final double HUB_TRAJECTORY_MAX_HEIGHT_FEET = 13;
    private final double PASSING_TRAJECTORY_MAX_HEIGHT_FEET = 5;

    private final Transform3d TURRET_TRANSFORM = new Transform3d(0, 0, 0, Rotation3d.kZero);

    public ShootingSuperstructure() {
        shooter = new ShooterSubsystem();
        turret = new TurretSubsystem();
        shotSensor = new CANrange(CAN_RANGE_ID);

        //TODO: Configure CANrange sensor
        shotSensor.getConfigurator().apply(shotSensorConfig);

        //Set the correct aiming locations
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            hubTarget = BLUE_HUB;
            passLeftTarget = PASS_BLUE_LEFT;
            passRightTarget = PASS_BLUE_RIGHT;
        }
        else {
            hubTarget = RED_HUB;
            passLeftTarget = PASS_RED_LEFT;
            passRightTarget = PASS_RED_RIGHT;
        }
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
     * Aim at our alliance's hub
     */
    public Command trackHub(Supplier<Pose3d> robotPose, Supplier<ChassisSpeeds> robotVelocity) {
        return trackTarget(() -> hubTarget, HUB_TRAJECTORY_MAX_HEIGHT_FEET, robotPose, robotVelocity);
    }

    /**
     * Aim to pass into our alliance area (dynamic, and based off of our field position)
     */
    public Command pass(Supplier<Pose3d> robotPose, Supplier<ChassisSpeeds> robotVelocity) {
        return trackTarget(() -> {
            //TODO: Implement pose checking to determine which target to aim at
            if (true) {
                return passLeftTarget;
            }
            else {
                return passRightTarget;
            }
        }, PASSING_TRAJECTORY_MAX_HEIGHT_FEET, robotPose, robotVelocity);
    }

    /**
     * Command that continually tracks a target with the hood, turret, and flywheel
     */
    private Command trackTarget(Supplier<Translation3d> target, double trajectoryHeight, Supplier<Pose3d> robotPose, Supplier<ChassisSpeeds> robotVelocity) {
        return run(() -> {
            ShotTrajectoryCalculator.update(
                robotPose.get().transformBy(TURRET_TRANSFORM),
                robotVelocity.get(),
                target.get(),
                trajectoryHeight
            );

            shooter.spinToRPM(() -> ShotTrajectoryCalculator.getTargetFlywheelRPM());
            shooter.setHoodPosition(() -> Units.degreesToRotations(ShotTrajectoryCalculator.getHoodAngle()));

            double turretTarget = Units.radiansToDegrees(robotPose.get().getRotation().getAngle()) - ShotTrajectoryCalculator.getTurretAngle();
            turret.rotateToAngle(() -> turretTarget);
        });
    }

    /**
     * @return If the shot sensor detects that a fuel hasn't been shot for MAX_SHOT_SPACING_SECONDS
     */
    public boolean isShooting() {
        return shotDebouncer.calculate(false);
    }

    @Override
    public void periodic() {
        DogLog.forceNt.log("ShootingSuperstructure/is_shooting", isShooting());
    }
}
