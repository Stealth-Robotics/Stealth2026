package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.CurrentAlliance;
import frc.robot.util.ShotParams;
import frc.robot.util.ShotTrajectoryCalculator;
import frc.robot.util.ZoneManager;

public class ShootingSuperstructure extends SubsystemBase {
    private ShooterState state = ShooterState.IDLE;
    private boolean applyIdle = false;

    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final TransferSubsystem transfer;

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelocitySupplier;

    //TODO: Find maximum time in between shots when rapidly shooting
    private final double MAX_SHOT_SPACING_SECONDS = 0.75;

    private final CANrange shotSensor;
    private final CANrangeConfiguration shotSensorConfig = new CANrangeConfiguration();

    private final double FUEL_DETECTED_THRESHOLD_INCHES = 0.5;

    private final Debouncer shotDebouncer = new Debouncer(MAX_SHOT_SPACING_SECONDS, DebounceType.kFalling);

    //TODO: Tune these values to actual goal
    private final double HUB_TRAJECTORY_MAX_HEIGHT_METERS = 4;
    private final double PASSING_TRAJECTORY_MAX_HEIGHT_METERS = 2.5;

    //TODO: Tune these values to actual goal
    private final ShotParams hub = new ShotParams(new Translation3d(4.645359992980957, 4.034599781036377, 1.8288), HUB_TRAJECTORY_MAX_HEIGHT_METERS);

    //TODO: Tune these values to actual targets
    private final ShotParams leftPass = new ShotParams(new Translation3d(), PASSING_TRAJECTORY_MAX_HEIGHT_METERS);
    private final ShotParams rightPass = new ShotParams(new Translation3d(), PASSING_TRAJECTORY_MAX_HEIGHT_METERS);

    //TODO: Get actual accurate CAD measurement
    private final Transform3d TURRET_TRANSFORM_METERS = new Transform3d(0.189, -0.2, 0.4, Rotation3d.kZero);

    private final int CAN_RANGE_ID = 15;

    public enum ShooterState {
        IDLE,
        PASSING,
        HUB_TRACKING
    }

    public ShootingSuperstructure(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotVelocitySupplier) {
        shooter = new ShooterSubsystem();
        turret = new TurretSubsystem();
        transfer = new TransferSubsystem();

        shotSensor = new CANrange(CAN_RANGE_ID);

        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;

        //Configure CANRange sensor
        shotSensorConfig.FovParams.FOVRangeX = 6.75;
        shotSensorConfig.FovParams.FOVRangeY = 6.75;
        shotSensorConfig.ToFParams.UpdateFrequency = 50;
        shotSensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRangeUserFreq;

        shotSensor.getConfigurator().apply(shotSensorConfig);
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public Command shoot() {
        return shooter.spinToRPM(() -> ShotTrajectoryCalculator.getTargetFlywheelRPM()).andThen(run(() -> {
            if (readyToShoot()) {
                transfer.spin().schedule();
                transfer.feed().schedule();
            }
            else {
                transfer.stopSpinning().schedule();
                transfer.stopFeeding().schedule();
            }
        }))
        .finallyDo(() -> {
            shooter.deactivateShooter().schedule();
            transfer.stopSpinning().schedule();
            transfer.stopFeeding().schedule();
        });
    }

    /**
     * Set the hood, turret, and flywheel to their homed/idle states (zeroed and unpowered)
     */
    private void idleSubsystems() {
        shooter.homeSubsystem().schedule();
        turret.homeSubsystem().schedule();
    }

    private void trackHub() {
        ShotParams params = allianceFlip(hub);
        Pose3d robotPose3d = new Pose3d(robotPoseSupplier.get());

        ShotTrajectoryCalculator.update(
            robotPose3d.transformBy(TURRET_TRANSFORM_METERS),
            robotVelocitySupplier.get(),
            params.target(),
            params.maxTrajectoryHeight()
        );

        shooter.setHoodDegrees(() -> ShotTrajectoryCalculator.getHoodAngle()).schedule();

        double turretTarget = Units.radiansToDegrees(robotPose3d.getRotation().getZ()) - ShotTrajectoryCalculator.getTurretAngle();
        turret.rotateToAngle(() -> turretTarget).schedule();
    }

    /**
     * Aim to pass into our alliance area (dynamic, based off of our field position)
     */
    private void pass() {
        ShotParams params;

        if (ZoneManager.inLeftPassingZone()) 
            params = allianceFlip(leftPass);
        else
            params = allianceFlip(rightPass);

        Pose3d robotPose3d = new Pose3d(robotPoseSupplier.get());

        ShotTrajectoryCalculator.update(
            robotPose3d.transformBy(TURRET_TRANSFORM_METERS),
            robotVelocitySupplier.get(),
            params.target(),
            params.maxTrajectoryHeight()
        );

        shooter.setHoodDegrees(() -> ShotTrajectoryCalculator.getHoodAngle()).schedule();

        double turretTarget = Units.radiansToDegrees(robotPose3d.getRotation().getZ()) - ShotTrajectoryCalculator.getTurretAngle();
        turret.rotateToAngle(() -> turretTarget).schedule();
    }

    /**
     * Make sure that we are in a shooting mode and the subsystems are within an acceptable tolerance
     */
    private boolean readyToShoot() {
        return shooter.isShooterAtVelocity() && turret.isTurretNearAngle();
    }

    /**
     * @return If the shot sensor detects that a fuel hasn't been shot for MAX_SHOT_SPACING_SECONDS
     */
    private boolean isShooting() {
        return shotDebouncer.calculate(Units.metersToInches(shotSensor.getDistance().getValueAsDouble()) < FUEL_DETECTED_THRESHOLD_INCHES);
    }

    /**
     * Flips the pose to the red alliance if we are indeed on the red alliance (blue is default otherwise)
     */
    private ShotParams allianceFlip(ShotParams original) {
        if (CurrentAlliance.get().equals(Alliance.Red)) {
            var rotated2d = original.target().toTranslation2d().rotateBy(Rotation2d.fromDegrees(180));

            return new ShotParams(
                new Translation3d(
                    rotated2d.getX(),
                    rotated2d.getY(),
                    original.target().getZ()
                ), 
                original.maxTrajectoryHeight()
            );
        }

        return original;
    }

    @Override
    public void periodic() {
        switch (state) {
            case IDLE -> {
                if (applyIdle) {
                    idleSubsystems();
                    applyIdle = false;
                }
            }

            case HUB_TRACKING -> {
                trackHub();
                applyIdle = true;
            }

            case PASSING -> {
                pass();
                applyIdle = true;
            }
        }

        DogLog.log("ShootingSuperstructure/state", state.name());
        DogLog.log("ShootingSuperstructure/is_shooting", isShooting());
    }
}
