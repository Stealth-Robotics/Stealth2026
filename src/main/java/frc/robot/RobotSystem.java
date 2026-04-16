package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.FieldPose;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSuperstructure.ShooterState;
import frc.robot.util.AllianceUtility;
import frc.robot.util.DogLogUtil;
import frc.robot.util.DrivingMode;
import frc.robot.util.LimelightConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.ShiftTracker;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;
import frc.robot.util.ZoneManager.FieldZone;
import frc.robot.util.ZoneManager;

public class RobotSystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;
    private final LEDSubsystem led;
    
    private final Field2d elasticField = new Field2d();

    //Allows us to disable certain logging for performance reasons
    private final boolean LOG_LIMELIGHTS = true;
    private final boolean LOG_SWERVE_DRIVE = true;
    private final boolean LOG_PDH = true;

    private DrivingMode currentDrivingMode = DrivingMode.NORMAL;
    private DrivingMode lastDrivingMode = DrivingMode.NORMAL;

    private double filteredX, filteredY, filteredTheta, lastFilteredX, lastFilteredY, lastFilteredTheta;

    private final SlewRateLimiter precisionXLimiter = new SlewRateLimiter(4.0), precisionYLimiter = new SlewRateLimiter(4.0);
    private final SlewRateLimiter precisionThetaLimiter = new SlewRateLimiter(10.0);

    private final AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    private final PowerDistribution pdh = new PowerDistribution(63, ModuleType.kRev);

    //Pose centered on the front of the hub to reset to if our vision goes haywire
    private final Pose2d ODOMETRY_RESET_POSE = new Pose2d(3.612, 4.027, Rotation2d.kZero);

    private long lastMs = 0;

    public RobotSystem(CommandXboxController driverController, CommandXboxController operatorController) {
        drive = TunerConstants.createDrivetrain();
        intake = new IntakeSubsystem();
        shooter = new ShootingSuperstructure(
            () -> drive.getPose(), 
            () -> drive.getFieldRelativeVelocity()
        );
        led = new LEDSubsystem(() -> ShiftTracker.hubIsActive());

        //Log the field + robot pose to Elastic
        SmartDashboard.putData("ElasticField", elasticField);

        ShiftTracker.shiftWarningTrigger.onTrue(led.blink());
    }

    public Command forceResetOdometry() {
        return new InstantCommand(() -> drive.resetPose(AllianceUtility.flipPose(ODOMETRY_RESET_POSE)));
    }

    public void configureIntake(DoubleSupplier rollerSpeed, BooleanSupplier deploy, BooleanSupplier retract, 
        BooleanSupplier fullAgitate) {

        Trigger deployTrigger = new Trigger(deploy);
        deployTrigger.onTrue(intake.deployCommand());

        Trigger retractTrigger = new Trigger(retract);
        retractTrigger.onTrue(intake.retractCommand());

        // Trigger automaticAgitateTrigger = new Trigger(() ->
        //     DriverStation.isTeleop() &&
        //     shooter.isShooting() &&
        //     intake.isDeployed() &&
        //     !deploy.getAsBoolean() &&
        //     !fullAgitate.getAsBoolean() &&
        //     !intake.isRetracting()
        // );
        // automaticAgitateTrigger.onTrue(intake.partialAgitate(() -> 0.25));

        Trigger fullAgitateTrigger = new Trigger(() -> fullAgitate.getAsBoolean() && !deploy.getAsBoolean());
        fullAgitateTrigger.whileTrue(intake.fullAgitate());

        Command intakeDefaultCommand = new RunCommand(
            () -> {
                if (!DriverStation.isAutonomous()) {
                    double targetRollerSpeed = rollerSpeed.getAsDouble();
                    intake.setRollerSpeed(targetRollerSpeed);
                }
            }, 
            intake
        );

        intake.setDefaultCommand(intakeDefaultCommand);
    }

    public Command dashboardHoodReset() {
        return shooter.dashboardHoodReset();
    }

    public Command shoot() {
        return shooter.shoot();
    }

    public Command clearTransfer() {
        return shooter.clearTransfer();
    }

    public void changeRPMOffset(int delta) {
        shooter.changeRPMOffset(delta);
    }

    private void updateShootingState() {
        FieldZone zone = ZoneManager.getZone();

        if (zone.equals(FieldZone.TRENCH))
            shooter.setState(ShooterState.TRENCH);
        else if (zone.equals(FieldZone.PASS))
            shooter.setState(ShooterState.PASS);
        else
            shooter.setState(ShooterState.HUB);
    }

    /**
     * @param x The supplier for driving the robot forward (field centric)
     * @param y The supplier for driving the robot sideways (field centric)
     * @param theta The supplier for rotating the robot
     */
    public void setDriveDefaultCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        drive.setDefaultCommand(
            drive.applyRequest(() -> {
                double xInput = x.getAsDouble(), yInput = y.getAsDouble(), thetaInput = theta.getAsDouble();
                
                //Change inputs for finer control around zero
                xInput = Math.copySign(Math.pow(xInput, 2), xInput);
                yInput = Math.copySign(Math.pow(yInput, 2), yInput);
                thetaInput = Math.copySign(Math.pow(thetaInput, 2), thetaInput);

                if (currentDrivingMode != lastDrivingMode) {
                    precisionXLimiter.reset(lastFilteredX);
                    precisionYLimiter.reset(lastFilteredY);
                    precisionThetaLimiter.reset(lastFilteredTheta);

                    lastDrivingMode = currentDrivingMode;
                }

                if (currentDrivingMode.equals(DrivingMode.PRECISION)) {
                    filteredX = precisionXLimiter.calculate(xInput);
                    filteredY = precisionYLimiter.calculate(yInput);
                    filteredTheta = precisionThetaLimiter.calculate(thetaInput);
                }
                else {
                    filteredX = xInput;
                    filteredY = yInput;
                    filteredTheta = thetaInput;
                }

                lastFilteredX = filteredX;
                lastFilteredY = filteredY;
                lastFilteredTheta = filteredTheta;

                double speed = currentDrivingMode.getSlowingFactor();

                return drive.fieldCentric
                    .withVelocityX(-filteredY * drive.MAX_SPEED * speed)
                    .withVelocityY(-filteredX * drive.MAX_SPEED * speed)
                    .withRotationalRate(-filteredTheta * drive.MAX_ANGULAR_RATE * speed);
            })
        );
    }

    public Command driveToPose(FieldPose targetPose) {
        return drive.goToPose(() -> targetPose);
    }

    public Command activatePrecisionDriving() {
        return new StartEndCommand(
            () -> {
                currentDrivingMode = DrivingMode.PRECISION;
            },
            () -> {
                currentDrivingMode = DrivingMode.NORMAL;
            }
        );
    }

    public Command seedFieldCentric() {
        return runOnce(() -> drive.seedFieldCentric());
    }

    public Autos getAutos() {
        return new Autos(
            drive,
            intake,
            shooter
        );
    }

    public double getRobotYawDegrees() {
        return drive.getPose().getRotation().getDegrees();
    }

    private void updateOdometry() {
        var driveSpeeds = drive.getState().Speeds;
        double linearSpeed = Math.hypot(driveSpeeds.vxMetersPerSecond, driveSpeeds.vyMetersPerSecond);

        if (linearSpeed < 3 && Math.abs(driveSpeeds.omegaRadiansPerSecond) < 2) {
            PoseEstimate bestEstimate = null;

            double robotYaw = drive.getState().Pose.getRotation().getDegrees();
            double robotYawRate = drive.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();

            for (String limelight : LimelightConstants.LIMELIGHTS) {
                LimelightHelpers.SetRobotOrientation(limelight, robotYaw, robotYawRate, 0, 0, 0, 0);

                var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
                if (isGoodPoseEstimate(estimate) && isBetterPoseEstimate(estimate, bestEstimate))
                    bestEstimate = estimate;
            }

            if (bestEstimate != null) {
                drive.addVisionMeasurement(
                    bestEstimate.pose,
                    bestEstimate.timestampSeconds,
                    LimelightConstants.STDDEVS
                );
            }
        }
    }

    private boolean isBetterPoseEstimate(PoseEstimate first, PoseEstimate second) {
        return second == null || first.tagCount > second.tagCount || first.avgTagArea > second.avgTagArea;
    }

    private boolean isGoodPoseEstimate(PoseEstimate poseEstimate) {
        if (
            poseEstimate == null || poseEstimate.pose == null || poseEstimate.pose.equals(Pose2d.kZero) 
            || !AllianceUtility.isWithinField(poseEstimate.pose) ||
            poseEstimate.tagCount <= LimelightConstants.MIN_TAG_COUNT ||
           (poseEstimate.tagCount == 1 && poseEstimate.avgTagDist >= LimelightConstants.MAX_SINGLE_TAG_DISTANCE) ||
           (poseEstimate.tagCount > 1 && poseEstimate.avgTagDist >= LimelightConstants.MAX_MULTI_TAG_DISTANCE)
        ) return false;
        
        for (RawFiducial tag : poseEstimate.rawFiducials) {
            if (tag.ambiguity >= LimelightConstants.MAX_TAG_AMBIGUITY) {
                return false;
            }
        }

        return true;
    }

    public void toggleDisabledLeds(boolean disable) {
        led.setIsDisabled(disable);
    }

    public void setLEDBrightness(double value) {
        led.setLEDBrightness(value);
    }

    public void resetFuelShotCount() {
        shooter.resetFuelShotCount();
    }

    @Override
    public void periodic() {
        var drivePose = drive.getPose();
        
        ZoneManager.updateRobotPose(drivePose);

        updateShootingState();

        //Update odometry with our Limelight's and also log everything
        updateOdometry();

        //Update the field's robot pose
        elasticField.setRobotPose(drivePose);

        DogLog.forceNt.log("Current Zone", ZoneManager.getZone().name());
        DogLog.forceNt.log("Driving Mode", currentDrivingMode.name());
        DogLog.forceNt.log("Drive Pose", drivePose);

        if (LOG_LIMELIGHTS) {
            for (String ll : LimelightConstants.LIMELIGHTS) {
                PoseEstimate m1Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);
                if (m1Pose != null) {
                    DogLog.log(ll + "/M1Pose", m1Pose.pose);

                    List<Pose3d> visibleTags = new ArrayList<>();
                    for (var tag : m1Pose.rawFiducials) {
                        tagFieldLayout.getTagPose(tag.id).ifPresent(visibleTags::add);
                    }

                    if (!visibleTags.isEmpty())
                        DogLog.log(ll + "/VisibleTags", visibleTags.toArray(new Pose3d[0]));
                }
            }
        }

        long currentMs = System.currentTimeMillis();
        if (currentMs - lastMs >= DogLogUtil.MOTOR_LOGGING_INTERVAL_MS) {
            if (LOG_SWERVE_DRIVE)
                logDriveStats();
            logStats();
            logPdhStats();
            lastMs = currentMs;
        }
    }

    private void logPdhStats() {
        if (LOG_PDH) {
            DogLog.log("PDH/TotalCurrent", pdh.getTotalCurrent());
            DogLog.log("PDH/Voltage", pdh.getVoltage());
            DogLog.log("PDH/Temperature", pdh.getTemperature());
        }
    }

    private void logStats() {
        var canStatus = RobotController.getCANStatus();
        DogLog.log("CAN/Utilization", canStatus.percentBusUtilization * 100);
        DogLog.log("CAN/TxError", canStatus.txFullCount);
        DogLog.log("CAN/RxError", canStatus.receiveErrorCount);
    }

    private void logDriveStats() {
        DogLog.log("Drive/ChassisSpeeds", drive.getRobotRelativeVelocity());
        DogLog.log("Drive/ModuleStates", drive.getModuleStates());
        DogLog.log("Drive/Rotation", drive.getPose().getRotation());

        for (var module : drive.getModules()) {
            DogLogUtil.logDouble("Drive/" + TunerConstants.getDeviceName(module.getDriveMotor().getDeviceID()) + "_Current",
                module.getDriveMotor().getSupplyCurrent(true).getValueAsDouble());
            DogLogUtil.logDouble("Drive/" + TunerConstants.getDeviceName(module.getSteerMotor().getDeviceID()) + "_Current",
                module.getSteerMotor().getSupplyCurrent(true).getValueAsDouble());
            DogLogUtil.logDouble("Drive/" + TunerConstants.getDeviceName(module.getDriveMotor().getDeviceID()) + "_Stator_Current",
                module.getDriveMotor().getStatorCurrent(true).getValueAsDouble());
            DogLogUtil.logDouble("Drive/" + TunerConstants.getDeviceName(module.getSteerMotor().getDeviceID()) + "_Stator_Current",
                module.getSteerMotor().getStatorCurrent(true).getValueAsDouble());
            DogLogUtil.logDouble("Drive/" + TunerConstants.getDeviceName(module.getDriveMotor().getDeviceID()) + "_Temperature_C",
                module.getDriveMotor().getDeviceTemp(true).getValueAsDouble());
            DogLogUtil.logDouble("Drive/" + TunerConstants.getDeviceName(module.getSteerMotor().getDeviceID()) + "_Temperature_C",
                module.getSteerMotor().getDeviceTemp(true).getValueAsDouble());
        }     
    }
}