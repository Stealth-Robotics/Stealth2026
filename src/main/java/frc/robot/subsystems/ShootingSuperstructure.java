package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.AllianceUtility;
import frc.robot.util.ShiftTracker;
import frc.robot.util.ShotParams;
import frc.robot.util.ShotCalculator;

public class ShootingSuperstructure extends SubsystemBase {
    private ShooterState state = ShooterState.IDLE;
    private boolean applyIdle = true;

    private PassingTarget passingTarget = PassingTarget.NONE;

    private boolean isShooting = false;

    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final TransferSubsystem transfer;

    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> robotVelocitySupplier;

    //The error of the turret if the target angle is beyond its limits
    public double turretLockError = 0;

    private final double SHOOTER_REVERSE_RPM = -2000;

    private final CANrange shotSensor;
    private final CANrangeConfiguration shotSensorConfig = new CANrangeConfiguration();

    private final double HUB_TRAJECTORY_MAX_HEIGHT_METERS = 3;
    private final double PASSING_TRAJECTORY_MAX_HEIGHT_METERS = 6;

    //The target pose that we are currently aiming at
    private Translation3d aimingTarget = Translation3d.kZero;

    private final ShotParams hub = new ShotParams(new Translation3d(4.645, 4.034, 1.828), HUB_TRAJECTORY_MAX_HEIGHT_METERS);

    private final ShotParams leftPass = new ShotParams(new Translation3d(0, 6.84, 0), PASSING_TRAJECTORY_MAX_HEIGHT_METERS);
    private final ShotParams middlePass = new ShotParams(new Translation3d(0, 4, 0), PASSING_TRAJECTORY_MAX_HEIGHT_METERS);
    private final ShotParams rightPass = new ShotParams(new Translation3d(0, 1.16, 0), PASSING_TRAJECTORY_MAX_HEIGHT_METERS);

    private final Transform3d TURRET_TRANSFORM_METERS = new Transform3d(0.19, -0.2, 0.5, Rotation3d.kZero);

    private final int CAN_RANGE_ID = 15;

    public enum ShooterState {
        IDLE,
        PASSING,
        HUB_TRACKING
    }

    public enum PassingTarget {
        NONE,
        LEFT,
        MIDDLE,
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
        shotSensorConfig.ToFParams.UpdateFrequency = 50;
        shotSensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRangeUserFreq;

        shotSensor.getConfigurator().apply(shotSensorConfig);
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public void setPassingTarget(PassingTarget newTarget) {
        passingTarget = newTarget;
    }

    public Command spinUp(double rpm) {
        return new InstantCommand(() -> shooter.spinToRPM(rpm));
    }

    public Command shoot() {
        return run(() -> {
            shooter.spinToRPM(ShotCalculator.getTargetFlywheelRPM());

            if (readyToShoot(() -> DriverStation.isAutonomous())) {
            // if(true){
                transfer.spin();
                transfer.feed();
            }
            else {
                transfer.stopSpinning();
                transfer.stopFeeding();
            }

            isShooting = true;
        })
        .finallyDo(() -> {
            shooter.coastShooter();
            transfer.stopSpinning();
            transfer.stopFeeding();

            isShooting = false;
        })
        .onlyWhile(() -> {
            if (SmartDashboard.getBoolean("Override ShiftTracker", false) || state.equals(ShooterState.PASSING))
                return true;

            return (state.equals(ShooterState.HUB_TRACKING) && ShiftTracker.canScore());
        });
    }

    public Command shootForTimeCommand(double time) {
        return new ParallelDeadlineGroup(shoot(), new WaitCommand(time));
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
            params.maxTrajectoryHeight()
        );

        shooter.setHoodDegrees(ShotCalculator.getHoodAngle());

        Rotation2d robotYaw = new Rotation2d(turretPose3d.getRotation().getZ());
        Rotation2d turretOffset = Rotation2d.fromDegrees(ShotCalculator.getTurretAngle());

        Rotation2d turretTargetRot = robotYaw.minus(turretOffset);

        turret.setTargetDegrees(turretTargetRot.getDegrees());

        calculateTurretLockError(turretTargetRot.getDegrees());

        aimingTarget = params.target();
    }

    /**
     * Aim to pass into our alliance area (dynamic, based off of our field position)
     */
    private void pass() {
        if (passingTarget.equals(PassingTarget.NONE)) {
            //Attempts to read the driver station location from the FMS and defaults to the MIDDLE if none is found
            passingTarget = PassingTarget.values()[DriverStation.getLocation().orElse(2) - 1];
        }

        ShotParams params = AllianceUtility.flipPose(
            (passingTarget.equals(PassingTarget.LEFT) ? leftPass : 
            passingTarget.equals(PassingTarget.MIDDLE) ? middlePass : rightPass)
        );
            
        Pose3d turretPose3d = new Pose3d(robotPoseSupplier.get()).transformBy(TURRET_TRANSFORM_METERS);

        ShotCalculator.update(
            turretPose3d,
            robotVelocitySupplier.get(),
            params.target(),
            params.maxTrajectoryHeight()
        );

        shooter.setHoodDegrees(ShooterSubsystem.MAX_HOOD_DEGREES);

        Rotation2d robotYaw = new Rotation2d(turretPose3d.getRotation().getZ());
        Rotation2d turretOffset = Rotation2d.fromDegrees(ShotCalculator.getTurretAngle());

        Rotation2d turretTargetRot = robotYaw.minus(turretOffset);

        turret.setTargetDegrees(turretTargetRot.getDegrees());

        calculateTurretLockError(turretTargetRot.getDegrees());

        aimingTarget = params.target();
    }

    private void calculateTurretLockError(double turretTarget) {
        turretLockError = (turretTarget > turret.MAX_TURRET_DEGREES) ? 
            turret.MAX_TURRET_DEGREES - turretTarget :
            (turretTarget < turret.MIN_TURRET_DEGREES) ? turret.MIN_TURRET_DEGREES - turretTarget  : 0;
    }

    /**
     * Make sure that we are in a shooting mode and the subsystems are within an acceptable tolerance
     */
    private boolean readyToShoot(BooleanSupplier override) {
        return override.getAsBoolean() || (!state.equals(ShooterState.IDLE) && shooter.isShooterAtVelocity() && turret.isReady());
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
        DogLog.log("ShootingSuperstructure/passing_target", passingTarget.name());
        DogLog.log("ShootingSuperstructure/aiming_target", aimingTarget);
    }
}
