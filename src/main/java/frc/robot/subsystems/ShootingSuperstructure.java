package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
    private ShooterState state = ShooterState.HOMED;

    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;

    //TODO: Find maximum time inbetween shots when rapidly shooting
    private final double MAX_SHOT_SPACING_SECONDS = 0.5;

    private final CANrange shotSensor;
    private final CANrangeConfiguration shotSensorConfig = new CANrangeConfiguration();

    //TODO: Tune distance
    private final double FUEL_DETECTED_THRESHOLD_INCHES = 0.5f;

    private final Debouncer shotDebouncer = new Debouncer(MAX_SHOT_SPACING_SECONDS, DebounceType.kFalling);

    private final String turretLimelight = "turret_limelight";

    public static final Translation3d BLUE_HUB = new Translation3d();
    public static final Translation3d RED_HUB = new Translation3d();

    public static final Translation3d PASS_BLUE_LEFT = new Translation3d();
    public static final Translation3d PASS_BLUE_RIGHT = new Translation3d();

    public static final Translation3d PASS_RED_LEFT = new Translation3d();
    public static final Translation3d PASS_RED_RIGHT = new Translation3d();

    private final double HUB_TRAJECTORY_MAX_HEIGHT_FEET = 13;
    private final double PASSING_TRAJECTORY_MAX_HEIGHT_FEET = 5;

    private final Transform3d TURRET_TRANSFORM = new Transform3d(0, 0, 0, Rotation3d.kZero);

    //TODO: Find actual CAN ID
    private final int CAN_RANGE_ID = 0;

    private enum ShooterState {
        HOMED,
        PASSING,
        HUB_TRACKING
    }

    public ShootingSuperstructure() {
        shooter = new ShooterSubsystem();
        turret = new TurretSubsystem();
        shotSensor = new CANrange(CAN_RANGE_ID);

        //Configure CANRange sensor
        shotSensorConfig.FovParams.FOVRangeX = 6.75;
        shotSensorConfig.FovParams.FOVRangeY = 6.75;
        shotSensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        
        shotSensor.getConfigurator().apply(shotSensorConfig);
    }

    /**
     * Reset the turret, hood, and flywheel back to their homed states (zeroed and unpowered)
     */
    public Command home() {
        return new SequentialCommandGroup(
            shooter.homeSubsystem(),
            turret.homeSubsystem()
        ).beforeStarting(() -> state = ShooterState.HOMED);
    }

    /**
     * Aim at our alliance's hub
     */
    public Command trackHub(Supplier<Pose3d> robotPose, Supplier<ChassisSpeeds> robotVelocity) {
        return trackTarget(() -> {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            if (alliance == Alliance.Blue) {
                return BLUE_HUB;
            }
            else {
                return RED_HUB;
            }
        }, HUB_TRAJECTORY_MAX_HEIGHT_FEET, robotPose, robotVelocity).beforeStarting(() -> state = ShooterState.HUB_TRACKING);
    }

    /**
     * Aim to pass into our alliance area (dynamic, and based off of our field position)
     */
    public Command pass(Supplier<Pose3d> robotPose, Supplier<ChassisSpeeds> robotVelocity) {
        return trackTarget(() -> {
            //TODO: Implement pose checking to determine which target to aim at
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

            if (alliance == Alliance.Blue) {
                return PASS_BLUE_LEFT;
            }
            else {
                return PASS_RED_LEFT;
            }
        }, PASSING_TRAJECTORY_MAX_HEIGHT_FEET, robotPose, robotVelocity).beforeStarting(() -> state = ShooterState.PASSING);
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

            shooter.spinToRPM(() -> ShotTrajectoryCalculator.getTargetFlywheelRPM()).schedule();
            shooter.setHoodPosition(() -> Units.degreesToRotations(ShotTrajectoryCalculator.getHoodAngle())).schedule();

            double turretTarget = Units.radiansToDegrees(robotPose.get().getRotation().getAngle()) - ShotTrajectoryCalculator.getTurretAngle();
            turret.rotateToAngle(() -> turretTarget).schedule();
        });
    }

    /**
     * Make sure that we are in a shooting mode and the subsystems are within an acceptable tolerance
     */
    public boolean isShootingValid() {
        return shooter.isShooterAtVelocity() && turret.isTurretAtAngle() && state != ShooterState.HOMED;
    }

    /**
     * @return If the shot sensor detects that a fuel hasn't been shot for MAX_SHOT_SPACING_SECONDS
     */
    public boolean isShooting() {
        return shotDebouncer.calculate(Units.metersToInches(shotSensor.getDistance().getValueAsDouble()) < FUEL_DETECTED_THRESHOLD_INCHES);
    }

    @Override
    public void periodic() {
        DogLog.forceNt.log("ShootingSuperstructure/is_shooting", isShooting());
    }
}
