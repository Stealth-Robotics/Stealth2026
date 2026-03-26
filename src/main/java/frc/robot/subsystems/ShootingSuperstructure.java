package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AllianceUtility;
import frc.robot.util.ShotParams;
import frc.robot.util.ShotCalculator;

public class ShootingSuperstructure extends SubsystemBase {
    private ShooterState state = ShooterState.IDLE;
    private boolean applyIdle = true;

    private int manualRPMOffset = 0;

    private double turretTargetDegrees = 0;

    private PassingTarget passingTarget = PassingTarget.RIGHT;

    private boolean isShooting = false;
    
    private boolean wasShotDetectedBefore = false;

    private int totalShots = 0;
    private int hubShots = 0;
    private int passShots = 0;

    private final Distance FUEL_DETECTED_DISTANCE_THRESHOLD = Inches.of(0.5);

    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final TransferSubsystem transfer;

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelocitySupplier;

    //The error of the turret if the target angle is beyond its limits
    public double turretLockError = 0;

    //Flag used to spin up for shooting and then forget checking rpms
    private boolean alreadySpinningAtTarget = false;

    private final double SHOOTER_REVERSE_RPM = -2000;

    private final CANrange shotSensor;
    private final CANrangeConfiguration shotSensorConfig = new CANrangeConfiguration();

    private final double HUB_TRAJECTORY_MAX_HEIGHT_METERS = 3;
    private final double PASSING_TRAJECTORY_MAX_HEIGHT_METERS = 6;

    //The target pose that we are currently aiming at
    private Translation3d aimingTarget = Translation3d.kZero;

    private final ShotParams hub = new ShotParams(new Translation3d(4.645, 4.034, 1.828), HUB_TRAJECTORY_MAX_HEIGHT_METERS);

    private final ShotParams leftPass = new ShotParams(new Translation3d(1, 5.75, 0), PASSING_TRAJECTORY_MAX_HEIGHT_METERS);
    private final ShotParams rightPass = new ShotParams(new Translation3d(1, 1.16, 0), PASSING_TRAJECTORY_MAX_HEIGHT_METERS);

    private final double FIELD_CENTER_Y_DIVIDER = 4.034663;

    private final Transform3d TURRET_TRANSFORM_METERS = new Transform3d(0.19, -0.2, 0.5, Rotation3d.kZero);

    private final int CAN_RANGE_ID = 15;

    public enum ShooterState {
        IDLE,
        TRENCH,
        PASSING,
        HUB_TRACKING
    }

    public enum PassingTarget {
        LEFT,
        RIGHT
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
        shotSensorConfig.withProximityParams(
            new ProximityParamsConfigs()
                .withProximityThreshold(FUEL_DETECTED_DISTANCE_THRESHOLD)
                .withProximityHysteresis(Inches.of(0.01))
          );

        shotSensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

        shotSensor.getConfigurator().apply(shotSensorConfig);

        // Probable should lower this a bit
        shotSensor.getIsDetected().setUpdateFrequency(200, 0.001); 

        //Reset the ShotCalculator's velocity filters 
        ShotCalculator.resetFilters();
    }

    public void changeRPMOffset(int delta) {
        manualRPMOffset += delta;
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public void resetFuelShotCount() {
        totalShots = 0;
        hubShots = 0;
        passShots = 0;
    }

    public Command spinUp(double rpm) {
        return new InstantCommand(() -> shooter.spinToRPM(rpm));
    }

    public Command shoot() {
        return run(() -> {
            shooter.spinToRPM(ShotCalculator.getTargetFlywheelRPM() + manualRPMOffset);

            if (!alreadySpinningAtTarget && shooter.isShooterAtVelocity()) {
                alreadySpinningAtTarget = true;
            }
            
            if (alreadySpinningAtTarget) {
                if (safeToShoot()) {
                    transfer.spin();
                    transfer.feed();
                }
                else {
                    transfer.stopSpinning();
                    transfer.stopFeeding();
                }
            }

            isShooting = true;
        })
        .finallyDo(() -> {
            shooter.coastShooter();
            transfer.stopSpinning();
            transfer.stopFeeding();

            isShooting = false;
            alreadySpinningAtTarget = false;
        })
        .onlyWhile(() -> {
            return state.equals(ShooterState.HUB_TRACKING) || state.equals(ShooterState.PASSING);
        });
    }

    public Command clearTransfer() {
        return run(() -> {
            transfer.reverseFeed();
            shooter.spinToRPM(SHOOTER_REVERSE_RPM);
        }).finallyDo(() -> { 
            transfer.stopFeeding();
            shooter.coastShooter();
        });
    }

    public void coastShooter() {
        shooter.coastShooter();
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
        Pose3d turretPose3d = new Pose3d(robotPoseSupplier.get()).transformBy(TURRET_TRANSFORM_METERS);

        ShotCalculator.update(
            turretPose3d,
            robotVelocitySupplier.get(),
            params.target(),
            params.maxTrajectoryHeight(),
            false
        );

        shooter.setHoodDegrees(ShotCalculator.getHoodAngle());

        Rotation2d robotYaw = new Rotation2d(turretPose3d.getRotation().getZ());
        Rotation2d turretOffset = Rotation2d.fromDegrees(ShotCalculator.getTurretAngle());

        Rotation2d turretTargetRot = robotYaw.minus(turretOffset);

        turret.setTargetDegrees(turretTargetRot.getDegrees());
        turretTargetDegrees = turretTargetRot.getDegrees();

        turretLockError = calculateTurretLockError(0);

        aimingTarget = params.target();
    }

    /**
     * Aim to pass into our alliance area (dynamic, based off of our field position)
     */
    private void pass() {
        Pose3d turretPose3d = new Pose3d(robotPoseSupplier.get()).transformBy(TURRET_TRANSFORM_METERS);

        //Calculate which side of the field to target for passing
        if (AllianceUtility.getAlliance().equals(Alliance.Blue)) {
            if (turretPose3d.getY() > FIELD_CENTER_Y_DIVIDER)
                passingTarget = PassingTarget.LEFT;
            else 
                passingTarget = PassingTarget.RIGHT;
        }
        else {
            if (turretPose3d.getY() < FIELD_CENTER_Y_DIVIDER)
                passingTarget = PassingTarget.LEFT;
            else 
                passingTarget = PassingTarget.RIGHT;
        }

        ShotParams params = AllianceUtility.flipPose(
            (passingTarget.equals(PassingTarget.LEFT) ? leftPass : rightPass)
        );

        ShotCalculator.update(
            turretPose3d,
            robotVelocitySupplier.get(),
            params.target(),
            params.maxTrajectoryHeight(),
            true
        );

        //Use the full hood angle to shoot as horizontally as possible
        shooter.setHoodDegrees(shooter.getMaxHoodDegrees());

        Rotation2d robotYaw = new Rotation2d(turretPose3d.getRotation().getZ());
        Rotation2d turretOffset = Rotation2d.fromDegrees(ShotCalculator.getTurretAngle());

        Rotation2d turretTargetRot = robotYaw.minus(turretOffset);

        turret.setTargetDegrees(turretTargetRot.getDegrees());
        turretTargetDegrees = turretTargetRot.getDegrees();

        turretLockError = calculateTurretLockError(0);

        aimingTarget = params.target();
    }

    public double calculateTurretLockError(double offset) {
        double target = turretTargetDegrees + offset;
        return (target > turret.MAX_TURRET_DEGREES) ?
            turret.MAX_TURRET_DEGREES - target :
            (target < turret.MIN_TURRET_DEGREES) ? turret.MIN_TURRET_DEGREES - target : 0;
    }

    /**
     * Makes sure we aren't shooting off the field or that we are going to definitely miss
     */
    private boolean safeToShoot() {
        boolean rotatingSlowEnough = robotVelocitySupplier.get().omegaRadiansPerSecond < Math.PI;
        return rotatingSlowEnough && turret.isReady();
    }

    public boolean isShooting() {
        return isShooting;
    }

    @Override
    public void periodic() {
        switch (state) {
            case IDLE -> {
                if (applyIdle) {
                    idleSubsystems();
                }
            }

            case TRENCH -> {
                shooter.setHoodDegrees(0);
                applyIdle = true;
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

        boolean shotDetected = shotSensor.getIsDetected().getValue();

        if (shotDetected && !wasShotDetectedBefore) {
            switch (state) {
                case HUB_TRACKING:
                    hubShots++;
                    break;
                case PASSING:
                    passShots++;
                    break;
                default:
                    break;
            }

            totalShots++;
        }

        wasShotDetectedBefore = shotDetected;

        //Log our shooting stats
        DogLog.log("ShootingSuperstructure/Hub_Shots_Total", hubShots);
        DogLog.log("ShootingSuperstructure/Pass_Shots_Total", passShots);
        DogLog.log("ShootingSuperstructure/Shot_Total", totalShots);

        DogLog.log("ShootingSuperstructure/state", state.name());
        DogLog.log("ShootingSuperstructure/RPM_Offset", manualRPMOffset);
    }
}
