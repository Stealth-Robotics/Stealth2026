package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AllianceUtility;
import frc.robot.util.ShotParams;
import frc.robot.util.ShotCalculator.SOTMResult;
import frc.robot.util.ShotCalculator;
import frc.robot.util.DogLogUtil;

public class ShootingSuperstructure extends SubsystemBase {
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final TransferSubsystem transfer;

    private final LinearFilter bpsFilter = LinearFilter.singlePoleIIR(0.1, Units.millisecondsToSeconds(20));
    private double lastShotTimestamp = 0.0;

    private ShooterState state = ShooterState.IDLE;
    private PassingTarget passingTarget = PassingTarget.RIGHT;

    private SOTMResult latestSOTMParameters = new SOTMResult(0, 0, 0, 0);

    //Allows us to manually offset the set RPMs during a match
    private int RPMOffset = 0;

    //The amount of time to wait before allowing shooting if the rpm tolerance code malfunctions
    private final double SECONDS_BEFORE_RPM_TOLERANCE_OVERRIDE = 1.5;

    //Used by the CANRange to determine whether a fuel is detected
    private final Distance FUEL_DETECTED_DISTANCE_THRESHOLD = Inches.of(3.8);

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelocitySupplier;

    //Flag used to spin up for shooting and then forget checking rpms
    private boolean alreadySpinningAtTarget = false;

    //RPMs used to clear the shooter of jammed fuel
    private final double SHOOTER_REVERSE_RPM = -2000;

    private final CANrange shotSensor;
    private final CANrangeConfiguration shotSensorConfig = new CANrangeConfiguration();

    private final double HUB_TRAJECTORY_MAX_HEIGHT_METERS = 3;
    private final double PASSING_TRAJECTORY_MAX_HEIGHT_METERS = 6;

    private final double FIELD_DIVIDER = 4.03;
    private final double PASSING_CENTER_DIVIDER_OFFSET = 0.85;

    private final ShotParams hub = new ShotParams(new Translation3d(4.645, 4.034, 1.828), HUB_TRAJECTORY_MAX_HEIGHT_METERS);
    private final ShotParams leftPass = new ShotParams(new Translation3d(1, 7.0, 0), PASSING_TRAJECTORY_MAX_HEIGHT_METERS);
    private final ShotParams rightPass = new ShotParams(new Translation3d(1, 1.16, 0), PASSING_TRAJECTORY_MAX_HEIGHT_METERS);

    private final Transform3d TURRET_TRANSFORM_METERS = new Transform3d(0.19, -0.2, 0.5, Rotation3d.kZero);

    private final Timer timeWeHaveBeenShooting = new Timer();

    private int totalShots = 0;
    private int hubShots = 0;
    private int passShots = 0;

    private boolean applyIdle = true;
    private boolean isShotRequested = false;
    private boolean isShooterActive = false;
    private boolean wasShotDetectedBefore = false;

    private final int CAN_RANGE_ID = 15;
    private final Timer lastShotTimer = new Timer();

    public enum ShooterState {
        IDLE,
        PASS,
        HUB,
        TRENCH
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
          );

        shotSensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

        shotSensor.getConfigurator().apply(shotSensorConfig);

        shotSensor.getIsDetected().setUpdateFrequency(100, 0.02);
    }

    public void changeRPMOffset(int delta) {
        RPMOffset += delta;
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
        return runOnce(() -> shooter.spinToRPM(rpm));
    }

    public Command dashboardHoodReset() {
        return shooter.dashboardHoodReset();
    }

    public Command shoot() {
        return run(() -> {
            isShotRequested = true;

            shooter.spinToRPM(latestSOTMParameters.rpm() + RPMOffset);

            if (!state.equals(ShooterState.TRENCH)) {
                shooter.setHoodDegrees(
                    (state.equals(ShooterState.PASS)) ? shooter.getMaxHoodDegrees() : latestSOTMParameters.hoodAngle()
                );
            }

            if (!alreadySpinningAtTarget) {
                if (shooter.isShooterAtVelocity() || timeWeHaveBeenShooting.hasElapsed(SECONDS_BEFORE_RPM_TOLERANCE_OVERRIDE))
                    alreadySpinningAtTarget = true;
            }
            
            if (alreadySpinningAtTarget) {
                if (safeToShoot()) {
                    isShooterActive = true;

                    transfer.spin(latestSOTMParameters.distance());
                    transfer.feed();
                }
                else {
                    isShooterActive = false;

                    transfer.stopSpinning();
                    transfer.stopFeeding();
                }
            }
        })
        .beforeStarting(() -> {
            timeWeHaveBeenShooting.restart();
        })
        .finallyDo(() -> {
            shooter.coastShooter();
            
            transfer.stopSpinning();
            transfer.stopFeeding();

            shooter.setHoodDegrees(0);

            timeWeHaveBeenShooting.stop();
            timeWeHaveBeenShooting.reset();

            alreadySpinningAtTarget = false;
            
            isShotRequested = false;
            isShooterActive = false;
        })
        .onlyWhile(() -> state.equals(ShooterState.HUB) || state.equals(ShooterState.PASS));
    }

    public Command clearTransfer() {
        return run(() -> {
            transfer.reverseFeed();
            transfer.reverseSpin();
            shooter.spinToRPM(SHOOTER_REVERSE_RPM);
        }).finallyDo(() -> { 
            transfer.stopFeeding();
            transfer.stopSpinning();
            shooter.coastShooter();
        });
    }

    public Command stopShooting() {
        return runOnce(() -> shooter.coastShooter());
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

        latestSOTMParameters = ShotCalculator.calculate(
            turretPose3d,
            robotVelocitySupplier.get(),
            params.target(),
            params.maxTrajectoryHeight(),
            false
        );

        Rotation2d robotYaw = robotPoseSupplier.get().getRotation();
        Rotation2d turretAngle = Rotation2d.fromDegrees(latestSOTMParameters.turretAngle());

        Rotation2d turretTargetRot = robotYaw.minus(turretAngle);

        turret.setTargetDegrees(turretTargetRot.getDegrees());
    }

    /**
     * Aim to pass into our alliance area (dynamic, based off of our field position)
     */
    private void pass() {
        Pose3d turretPose3d = new Pose3d(robotPoseSupplier.get()).transformBy(TURRET_TRANSFORM_METERS);

        passingTarget = calculatePassingTarget(turretPose3d.toPose2d());

        ShotParams params = AllianceUtility.flipPose(
            (passingTarget.equals(PassingTarget.LEFT) ? leftPass : rightPass)
        );

        latestSOTMParameters = ShotCalculator.calculate(
            turretPose3d,
            robotVelocitySupplier.get(),
            params.target(),
            params.maxTrajectoryHeight(),
            true
        );

        Rotation2d robotYaw = robotPoseSupplier.get().getRotation();
        Rotation2d turretAngle = Rotation2d.fromDegrees(latestSOTMParameters.turretAngle());

        Rotation2d turretTargetRot = robotYaw.minus(turretAngle);

        turret.setTargetDegrees(turretTargetRot.getDegrees());
    }

    private PassingTarget calculatePassingTarget(Pose2d turretPose) {
        int driverStation = DriverStation.getLocation().orElse(1);
        boolean preferLeft = driverStation == 0 || driverStation == 1;

        double turretY = turretPose.getY();
        if (AllianceUtility.getAlliance().equals(Alliance.Red)) {
            turretY = AllianceUtility.FIELD_WIDTH_METERS - turretY;
        }

        double threshold = FIELD_DIVIDER + (preferLeft ? -PASSING_CENTER_DIVIDER_OFFSET : PASSING_CENTER_DIVIDER_OFFSET);

        if (turretY > threshold) return PassingTarget.LEFT;
        else return PassingTarget.RIGHT;
    }

    private boolean safeToShoot() {
        return turret.isReady();
    }

    public boolean isShooting() {
        return isShooterActive;
    }

    private boolean isShotDetected() {
        double dist = shotSensor.getDistance(true).getValue().in(Inches);
        return dist < FUEL_DETECTED_DISTANCE_THRESHOLD.in(Inches);
    }

    private void updateShotCounting() {
        // Only run the timer when we are actively shooting.
        if (isShooting() && !lastShotTimer.isRunning()) {
            lastShotTimer.start();
        } 
        else if (!isShooting() && lastShotTimer.isRunning()) {
            lastShotTimer.reset();
            lastShotTimer.stop();
        }

        boolean shotDetected = isShotDetected();

        if (shotDetected && !wasShotDetectedBefore) {
            lastShotTimer.restart();

            switch (state) {
                case HUB:
                    hubShots++; 
                    break;
                case PASS:
                    passShots++;
                    break;
                default:
                    break;
            }

            totalShots++;

            //Calculate BPS
            double now = Timer.getFPGATimestamp();
            double timeSinceLastShot = now - lastShotTimestamp;

            if (timeSinceLastShot > 0) {
                bpsFilter.calculate(1.0 / timeSinceLastShot);
            }

            lastShotTimestamp = now;
        }

        wasShotDetectedBefore = shotDetected;
    }

    @Override
    public void periodic() {
        //Keep the hood down unless we are shooting
        if (!isShotRequested) shooter.setHoodDegrees(0);
      
        switch (state) {
            case IDLE -> {
                if (applyIdle) {
                    idleSubsystems();
                }
            }

            case TRENCH -> {
                //Keep hood down while in the trench
                shooter.setHoodDegrees(0);
                trackHub();
            }

            case HUB -> {
                trackHub();
                applyIdle = true;
            }

            case PASS -> {
                pass();
                applyIdle = true;
            }
        }

        updateShotCounting();

        DogLogUtil.logDouble("ShootingSuperstructure/BPS", bpsFilter.lastValue());

        //Log our shooting counts
        DogLog.log("ShootingSuperstructure/Hub_Shots_Total", hubShots);
        DogLog.log("ShootingSuperstructure/Pass_Shots_Total", passShots);
        DogLog.log("ShootingSuperstructure/Shot_Total", totalShots);

        DogLog.log("ShootingSuperstructure/isShootingRequested", isShotRequested);
        DogLog.log("ShootingSuperstructure/isShootingActive", isShooterActive);

        DogLog.forceNt.log("ShootingSuperstructure/state", state.name());
        DogLog.forceNt.log("ShootingSuperstructure/RPM_Offset", RPMOffset);
    }
}
