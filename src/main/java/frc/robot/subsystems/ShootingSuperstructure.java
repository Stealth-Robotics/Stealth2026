package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.AllianceUtility;
import frc.robot.util.ShiftTracker;
import frc.robot.util.ShotParams;
import frc.robot.util.ShotTrajectoryCalculator;
import frc.robot.util.ShotTrajectoryCalculator.ShotParameters;
import frc.robot.util.ZoneManager;
import frc.robot.util.ZoneManager.FieldZone;

public class ShootingSuperstructure extends SubsystemBase {
    private ShooterState state = ShooterState.IDLE;
    private boolean applyIdle = true;

    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final TransferSubsystem transfer;

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelocitySupplier;

    // The error of the turret if the target angle is beyond its limits
    public double turretLockError = 0;

    // Tracks whether the last computed shot was within a valid distance range
    private boolean lastShotValid = false;

    //TODO: Find maximum time in between shots when rapidly shooting
    private final double MAX_SHOT_SPACING_SECONDS = 0.75;

    private final double SHOOTER_REVERSE_RPM = -2000;

    private final CANrange shotSensor;
    private final CANrangeConfiguration shotSensorConfig = new CANrangeConfiguration();

    private final double FUEL_DETECTED_THRESHOLD_INCHES = 0.5;

    private final Debouncer shotDebouncer = new Debouncer(MAX_SHOT_SPACING_SECONDS, DebounceType.kFalling);

    private final ShotParams hub = new ShotParams(new Translation2d(4.645, 4.034));

    private final ShotParams leftPass = new ShotParams(new Translation2d(1.098, 6.84));
    private final ShotParams rightPass = new ShotParams(new Translation2d(1.098, 1.16));

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
        return run(() -> {
            if (readyToShoot()) {
                transfer.spin();
                transfer.feed();
            }
            else {
                transfer.stopSpinning();
                transfer.stopFeeding();
            }

        })
        .finallyDo(() -> {
            shooter.coastShooter();
            transfer.stopSpinning();
            transfer.stopFeeding();
        })
        .onlyWhile(() -> {
            if (SmartDashboard.getBoolean("Override ShiftTracker", false))
                return true;

            return (state.equals(ShooterState.HUB_TRACKING) && ShiftTracker.canScore()) || state.equals(ShooterState.PASSING);
        });
    }

    public Command clearTransfer() {
        return run(() -> {
            transfer.reverseFeed();
            shooter.spinToRPM(SHOOTER_REVERSE_RPM);
        }).finallyDo(() -> { 
            transfer.stopSpinning();
            shooter.coastShooter();
        });
    }

    public Command autonomousShoot() {
        Command stopCondition = new WaitCommand(MAX_SHOT_SPACING_SECONDS).andThen(new WaitUntilCommand(() -> !isShooting()));
        return shoot().withDeadline(stopCondition);
    }

    /**
     * Set the hood, turret, and flywheel to their homed/idle states (zeroed and unpowered)
     */
    private void idleSubsystems() {
        shooter.coastShooter();
        shooter.setHoodDegrees(0);

        turret.homeTurret();
    }

    private void trackHub() {
        ShotParams params = AllianceUtility.flipPose(hub);
        Pose2d robotPose = robotPoseSupplier.get();

        ShotParameters shot = ShotTrajectoryCalculator.getInstance().calculate(
            robotPose,
            robotVelocitySupplier.get(),
            params.target()
        );

        lastShotValid = shot.isValid();
        shooter.spinToRPM(shot.flywheelRPM());
        shooter.setHoodDegrees(shot.hoodAngleDegrees());

        double turretTarget = robotPose.getRotation().getDegrees() - shot.turretAngleDegrees();
        turret.setTargetDegrees(turretTarget);

        calculateTurretLockError(turretTarget);
    }

    /**
     * Aim to pass into our alliance area (dynamic, based off of our field position)
     */
    private void pass() {
        ShotParams params =
            ZoneManager.getZone().equals(FieldZone.LEFT_PASS) ?
            AllianceUtility.flipPose(leftPass) : AllianceUtility.flipPose(rightPass);

        Pose2d robotPose = robotPoseSupplier.get();

        ShotParameters shot = ShotTrajectoryCalculator.getInstance().calculate(
            robotPose,
            robotVelocitySupplier.get(),
            params.target()
        );

        lastShotValid = shot.isValid();
        shooter.spinToRPM(shot.flywheelRPM());
        shooter.setHoodDegrees(shot.hoodAngleDegrees());

        double turretTarget = robotPose.getRotation().getDegrees() - shot.turretAngleDegrees();
        turret.setTargetDegrees(turretTarget);

        calculateTurretLockError(turretTarget);
    }

    private void calculateTurretLockError(double turretTarget) {
        turretLockError = (turretTarget > turret.MAX_TURRET_DEGREES) ? 
            turret.MAX_TURRET_DEGREES - turretTarget :
            (turretTarget < turret.MIN_TURRET_DEGREES) ? turret.MIN_TURRET_DEGREES - turretTarget  : 0;
    }

    /**
     * Make sure that we are in a shooting mode, within a valid shot distance,
     * and the subsystems are within an acceptable tolerance.
     */
    private boolean readyToShoot() {
        return !state.equals(ShooterState.IDLE) && lastShotValid && shooter.isShooterAtVelocity() && turret.isReady();
    }

    /**
     * @return If the shot sensor detects that a fuel hasn't been shot for MAX_SHOT_SPACING_SECONDS
     */
    private boolean isShooting() {
        return shotDebouncer.calculate(Units.metersToInches(shotSensor.getDistance().getValueAsDouble()) < FUEL_DETECTED_THRESHOLD_INCHES);
    }

    @Override
    public void periodic() {
        switch (state) {
            case IDLE -> {
                if (applyIdle) {
                    idleSubsystems();
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
