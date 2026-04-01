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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AllianceUtility;
import frc.robot.util.ShotParams;
import frc.robot.util.ShotCalculator;

public class ShootingSuperstructure extends SubsystemBase {
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final TransferSubsystem transfer;

    private ShooterState state = ShooterState.IDLE;
    private PassingTarget passingTarget = PassingTarget.RIGHT;

    //Allows us to manually offset the set RPMs during a match
    private int RPMOffset = 0;

    //Prevents us from shooting if we are tiled enough to miss our target
    private final double MAX_PITCH_DEGREES = 5;
    private final double MAX_ROLL_DEGREES = 8;

    //Prevents us from shooting if we are moving/rotating too fast to hit our target (m/s, m/s, rad/s)
    private final double[] MAX_ROBOT_SHOOTING_VELOCITY = {2.0, 2.0, Math.PI};

    //Prevents us from shooting if we are accelerating too fast to track our target (m/s^2, m/s^2, rad/s^2)
    private final double[] MAX_ROBOT_SHOOTING_ACCELERATION = {1.5, 1.5, Math.PI};

    //Used by the CANRange to determine whether a fuel is detected
    private final Distance FUEL_DETECTED_DISTANCE_THRESHOLD = Inches.of(0.5);

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelocitySupplier;
    private final Supplier<Rotation3d> robotRotationSupplier;

    //Flag used to spin up for shooting and then forget checking rpms
    private boolean alreadySpinningAtTarget = false;

    //RPMs used to clear the shooter of jammed fuel
    private final double SHOOTER_REVERSE_RPM = -2000;

    private final CANrange shotSensor;
    private final CANrangeConfiguration shotSensorConfig = new CANrangeConfiguration();

    private final double HUB_TRAJECTORY_MAX_HEIGHT_METERS = 3;
    private final double PASSING_TRAJECTORY_MAX_HEIGHT_METERS = 6;

    //Used to determine which side of the field to pass towards
    private final double FIELD_CENTER_Y_DIVIDER = 4.034663;

    private final ShotParams hub = new ShotParams(new Translation3d(4.645, 4.034, 1.828), HUB_TRAJECTORY_MAX_HEIGHT_METERS);

    private final ShotParams leftPass = new ShotParams(new Translation3d(1, 5.75, 0), PASSING_TRAJECTORY_MAX_HEIGHT_METERS);
    private final ShotParams rightPass = new ShotParams(new Translation3d(1, 1.16, 0), PASSING_TRAJECTORY_MAX_HEIGHT_METERS);

    private final Transform3d TURRET_TRANSFORM_METERS = new Transform3d(0.19, -0.2, 0.5, Rotation3d.kZero);

    private int totalShots = 0;
    private int hubShots = 0;
    private int passShots = 0;

    private boolean applyIdle = true;
    private boolean isShooting = false;
    private boolean wasShotDetectedBefore = false;

    private double[] currentRobotAccel = {0.0, 0.0, 0.0};
    private ChassisSpeeds previousRobotVelo = new ChassisSpeeds();

    private final int CAN_RANGE_ID = 15;

    public enum ShooterState {
        IDLE,
        PASSING,
        HUB_TRACKING
    }

    public enum PassingTarget {
        LEFT,
        RIGHT
    }

    public ShootingSuperstructure(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotVelocitySupplier, Supplier<Rotation3d> robotRotationSupplier) {
        shooter = new ShooterSubsystem();
        turret = new TurretSubsystem();
        transfer = new TransferSubsystem();

        shotSensor = new CANrange(CAN_RANGE_ID);

        this.robotPoseSupplier = robotPoseSupplier;
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.robotRotationSupplier = robotRotationSupplier;

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

        shotSensor.getIsDetected().setUpdateFrequency(100, 0.005);
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
        return new InstantCommand(() -> shooter.spinToRPM(rpm));
    }

    public Command dashboardHoodReset() {
        return shooter.dashboardHoodReset();
    }

    public Command shoot() {
        return run(() -> {
            shooter.spinToRPM(ShotCalculator.getTargetFlywheelRPM() + RPMOffset);

            shooter.setHoodDegrees(
                (state.equals(ShooterState.PASSING)) ? shooter.getMaxHoodDegrees() : ShotCalculator.getHoodAngle()
            );

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

        Rotation2d robotYaw = new Rotation2d(turretPose3d.getRotation().getZ());
        Rotation2d turretOffset = Rotation2d.fromDegrees(ShotCalculator.getTurretAngle());

        Rotation2d turretTargetRot = robotYaw.minus(turretOffset);

        turret.setTargetDegrees(turretTargetRot.getDegrees());
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

        Rotation2d robotYaw = new Rotation2d(turretPose3d.getRotation().getZ());
        Rotation2d turretOffset = Rotation2d.fromDegrees(ShotCalculator.getTurretAngle());

        Rotation2d turretTargetRot = robotYaw.minus(turretOffset);

        turret.setTargetDegrees(turretTargetRot.getDegrees());
    }

    /**
     * Makes sure we aren't shooting off the field or that we are going to definitely miss
     */
    private boolean safeToShoot() {
        boolean isVelocityBelowThreshold = 
            Math.abs(robotVelocitySupplier.get().vxMetersPerSecond) < MAX_ROBOT_SHOOTING_VELOCITY[0] &&
            Math.abs(robotVelocitySupplier.get().vyMetersPerSecond) < MAX_ROBOT_SHOOTING_VELOCITY[1] &&
            Math.abs(robotVelocitySupplier.get().omegaRadiansPerSecond) < MAX_ROBOT_SHOOTING_VELOCITY[2];

        boolean isAccelBelowThreshold = 
            Math.abs(currentRobotAccel[0]) < MAX_ROBOT_SHOOTING_ACCELERATION[0] &&
            Math.abs(currentRobotAccel[1]) < MAX_ROBOT_SHOOTING_ACCELERATION[1] &&
            Math.abs(currentRobotAccel[2]) < MAX_ROBOT_SHOOTING_ACCELERATION[2];

        boolean isRobotLevelEnough = 
            Math.abs(robotRotationSupplier.get().getX()) < MAX_ROLL_DEGREES &&
            Math.abs(robotRotationSupplier.get().getY()) < MAX_PITCH_DEGREES;

        return isVelocityBelowThreshold && isAccelBelowThreshold && isRobotLevelEnough && turret.isReady();
    }

    public boolean isShooting() {
        return isShooting;
    }

    private void calculateRobotAccel() {
        var robotVelo = robotVelocitySupplier.get();

        currentRobotAccel[0] = (robotVelo.vxMetersPerSecond - previousRobotVelo.vxMetersPerSecond) / Units.millisecondsToSeconds(20);
        currentRobotAccel[1] = (robotVelo.vyMetersPerSecond - previousRobotVelo.vyMetersPerSecond) / Units.millisecondsToSeconds(20);
        currentRobotAccel[2] = (robotVelo.omegaRadiansPerSecond - previousRobotVelo.omegaRadiansPerSecond) / Units.millisecondsToSeconds(20);

        previousRobotVelo.vxMetersPerSecond = robotVelo.vxMetersPerSecond;
        previousRobotVelo.vyMetersPerSecond = robotVelo.vyMetersPerSecond;
        previousRobotVelo.omegaRadiansPerSecond = robotVelo.omegaRadiansPerSecond;
    }

    @Override
    public void periodic() {
        //Keep the hood down unless we are shooting
        if (!isShooting()) shooter.setHoodDegrees(0);

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

        calculateRobotAccel();

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

        DogLog.forceNt.log("ShootingSuperstructure/state", state.name());
        DogLog.forceNt.log("ShootingSuperstructure/RPM_Offset", RPMOffset);
    }
}
