package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
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
import frc.robot.auto.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSuperstructure.ShooterState;
import frc.robot.util.AllianceUtility;
import frc.robot.util.DogLogUtil;
import frc.robot.util.DrivingMode;
import frc.robot.util.LimelightConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;
import frc.robot.util.ZoneManager.FieldZone;
import frc.robot.util.ZoneManager;

public class RobotSystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;
    
    private final Field2d elasticField = new Field2d();

    //Allows us to disable certain logging for performance reasons
    private final boolean LOG_LIMELIGHTS = true;
    private final boolean LOG_SWERVE_DRIVE = false;
    private final boolean LOG_PDH = false;
    private final boolean LOG_PIGEON = false;
    private final boolean LOG_RIO_CAN = false;

    private DrivingMode currentDrivingMode = DrivingMode.NORMAL;
    private DrivingMode lastDrivingMode = DrivingMode.NORMAL;

    private double filteredX, filteredY, filteredTheta, lastFilteredX, lastFilteredY, lastFilteredTheta;

    private final SlewRateLimiter precisionXLimiter = new SlewRateLimiter(4.0), precisionYLimiter = new SlewRateLimiter(4.0);
    private final SlewRateLimiter precisionThetaLimiter = new SlewRateLimiter(10.0);

    private final AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    private final PowerDistribution pdh = new PowerDistribution(63, ModuleType.kRev);

    //Pose centered on the front of the hub to reset to if our vision goes haywire
    private final Pose2d ODOMETRY_RESET_POSE = new Pose2d(3.612, 4.027, Rotation2d.kZero);

    //Maximum jump size that the gyro can make in 20ms (if it is greater than it is highly sus)
    private static final double MAX_GYRO_JUMP_DEGREES = 15.0; //TODO: Tune to actual value
    
    private static final double MAX_VISION_ROTATION_ERROR_DEGREES = 45.0; //TODO: Tune to actual value

    private Rotation2d lastGoodGyroReading = Rotation2d.kZero;

    private boolean gyroReadingRejected = false; 
    private boolean hasValidGyroReading = false;

    private long lastLowPriLogMs = 0;
    private long lastLimelightLogMs = 0;
    private long lastHighPriStatsLogMs = 0;

    public RobotSystem(CommandXboxController driverController, CommandXboxController operatorController) {
        drive = TunerConstants.createDrivetrain();
        intake = new IntakeSubsystem();
        shooter = new ShootingSuperstructure(
            () -> drive.getPose(),
            () -> drive.getFieldRelativeVelocity()
        );

        //Log the field + robot pose to Elastic
        SmartDashboard.putData("ElasticField", elasticField);
    }

    public Command forceResetOdometry() {
        return new InstantCommand(() -> {
            Pose2d resetPose = AllianceUtility.flipPose(ODOMETRY_RESET_POSE);
            drive.resetPose(resetPose);

            lastGoodGyroReading = resetPose.getRotation();
            hasValidGyroReading = true;
        });
    }

    public Command seedFieldCentric() {
        return runOnce(() -> {
            drive.seedFieldCentric();

            lastGoodGyroReading = drive.getPose().getRotation();
            hasValidGyroReading = true;
        });
    }

    public void configureIntake(DoubleSupplier rollerSpeed, BooleanSupplier deploy, BooleanSupplier retract, 
        BooleanSupplier quickAgitate, BooleanSupplier fullAgitate) {

        Trigger deployTrigger = new Trigger(deploy);
        deployTrigger.onTrue(intake.deployCommand());

        Trigger retractTrigger = new Trigger(retract);
        retractTrigger.onTrue(intake.retractCommand());

        //TODO: Comment out if auto issues (auto not shooting or stopping mid-auto)

        Trigger raiseOnBumpTrigger = new Trigger(() -> 
            ZoneManager.inBumpZone() &&
            intake.isDeployed() &&
            !deploy.getAsBoolean()
        );
        raiseOnBumpTrigger.onTrue(new InstantCommand(() -> intake.safe()));

        Trigger lowerWhenNotOnBumpTrigger = new Trigger(() ->
            // !DriverStation.isAutonomous() &&
            !ZoneManager.inBumpZone() &&
            intake.isSafe() &&
            !intake.isRetracting() &&
            !quickAgitate.getAsBoolean() &&
            !fullAgitate.getAsBoolean() &&
            !deploy.getAsBoolean()
        );
        lowerWhenNotOnBumpTrigger.onTrue(new InstantCommand(() -> intake.deploy()));

        Trigger quickAgitateTrigger = new Trigger(() -> quickAgitate.getAsBoolean() && !deploy.getAsBoolean());
        quickAgitateTrigger.whileTrue(intake.quickAgitate(() -> 0.5).repeatedly());

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
        var robotSpeed = drive.getFieldRelativeVelocity();

        boolean rotatingSlowEnough = 
            Math.abs(robotSpeed.omegaRadiansPerSecond) < LimelightConstants.MAX_ANGULAR_VELO_RADIANS_PER_SECOND;
        boolean drivingSlowEnough = 
            Math.hypot(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond) < LimelightConstants.MAX_VELO_METERS_PER_SECOND;

        gyroReadingRejected = false;
        
        if (!rotatingSlowEnough || !drivingSlowEnough)
            return;

        Rotation2d rawRobotRotation = drive.getPose().getRotation(); 
        Rotation2d robotRotation = getTrustedGyroRotation(rawRobotRotation);

        for (String limelight : LimelightConstants.LIMELIGHTS) {
            LimelightHelpers.SetRobotOrientation(limelight, robotRotation.getDegrees(), 0, 0, 0, 0, 0);
            var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

            if (isGoodPoseEstimate(mt2)) {
                Optional<Pose2d> odometryAtVisionTime = drive.samplePoseAt(mt2.timestampSeconds);

                if (odometryAtVisionTime.isEmpty())
                    continue;

                Pose2d expectedPose = odometryAtVisionTime.get();
                double rotationError = Math.abs(mt2.pose.getRotation().minus(expectedPose.getRotation()).getDegrees());

                if (rotationError < MAX_VISION_ROTATION_ERROR_DEGREES)
                    drive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, LimelightConstants.MT2_STDDEVS);
            }
        }    
    }

    private boolean isGoodPoseEstimate(PoseEstimate poseEstimate) {
        boolean isBadEstimate = 
            poseEstimate == null ||
            poseEstimate.pose == null ||
            poseEstimate.rawFiducials == null ||
            poseEstimate.tagCount < LimelightConstants.MIN_TAG_COUNT ||
            poseEstimate.pose.equals(Pose2d.kZero) ||
            !AllianceUtility.isWithinField(poseEstimate.pose) ||
           (poseEstimate.tagCount == 1 && poseEstimate.avgTagDist >= LimelightConstants.MAX_SINGLE_TAG_DISTANCE) ||
           (poseEstimate.tagCount > 1 && poseEstimate.avgTagDist >= LimelightConstants.MAX_MULTI_TAG_DISTANCE);

        if (isBadEstimate) return false;
        
        if (poseEstimate.tagCount <= 1)
            for (RawFiducial tag : poseEstimate.rawFiducials)
                if (tag.ambiguity >= LimelightConstants.MAX_TAG_AMBIGUITY) return false;
        
        return true;
    }

    private Rotation2d getTrustedGyroRotation(Rotation2d rawRotation) {
        gyroReadingRejected = false;

        if (!hasValidGyroReading) { 
            lastGoodGyroReading = rawRotation;
            hasValidGyroReading = true;
            
            return rawRotation;
        }

        double rotationChangeDegrees = Math.abs(rawRotation.minus(lastGoodGyroReading).getDegrees());
        
        //Unacceptable magnitude of a jump, so we reject it
        if (rotationChangeDegrees > MAX_GYRO_JUMP_DEGREES) {
            gyroReadingRejected = true;
            
            //Reset the pigeon with last valid reading
            drive.recoverGyro(lastGoodGyroReading);

            return lastGoodGyroReading;
        }

        //Gyro reading must be sensible so we accept it :)
        lastGoodGyroReading = rawRotation;
        return rawRotation;
    }

    public void resetAfterAuto() {
        CommandScheduler.getInstance().schedule(
            intake.stopRollers(),
            shooter.stopShooting()
        );
    }

    public void resetFuelShotCount() {
        shooter.resetFuelShotCount();
    }

    public DriveSubsystem getDrive() {
        return drive;
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

        // All logging of data
        logStats(drivePose);
    }

    private void logStats(Pose2d drivePose) {
        long currentMs = System.currentTimeMillis();
        if (currentMs - lastHighPriStatsLogMs >= DogLogUtil.HIGH_PRI_STATS_LOGGING_INTERVAL) {
            DogLog.forceNt.log("Current Zone", ZoneManager.getZone().name());
            DogLog.forceNt.log("Driving Mode", currentDrivingMode.name());
            DogLog.forceNt.log("Drive Pose", drivePose);       
            lastHighPriStatsLogMs = currentMs;
        }

        if (currentMs - lastLowPriLogMs >= DogLogUtil.LOW_PRI_LOGGING_INTERVAL_MS) {
            logPdhStats();
            logCanStats();
            logDriveStats();
            logPigeonStats();
            lastLowPriLogMs = currentMs;
        }

        logLimelightStats();
    }   

    private void logLimelightStats() {
        if (!LOG_LIMELIGHTS) return;

        long currentMs = System.currentTimeMillis();
        if (currentMs - lastLimelightLogMs < DogLogUtil.LIMELIGHT_LOGGING_INTERVAL) return;
        
        lastLimelightLogMs = currentMs;

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

    private void logPigeonStats() {
        if (!LOG_PIGEON) return;

        DogLog.log("Pigeon/Total Yaw", drive.getPigeon2().getYaw().getValueAsDouble());
        DogLog.log("Pigeon/Accel Exceeded", drive.getPigeon2().getFault_SaturatedAccelerometer().getValue());
        DogLog.log("Pigeon/Boot While Enabled", drive.getPigeon2().getFault_BootDuringEnable().getValue());
        DogLog.log( "Pigeon/GyroReadingRejected", gyroReadingRejected); 
        DogLog.log( "Pigeon/TrustedRotation", lastGoodGyroReading);
    }

    private void logPdhStats() {
        if (!LOG_PDH) return;

        DogLog.log("PDH/TotalCurrent", pdh.getTotalCurrent());
        DogLog.log("PDH/Voltage", pdh.getVoltage());
        DogLog.log("PDH/Temperature", pdh.getTemperature());
    }

    private void logCanStats() {
        if (!LOG_RIO_CAN) return;
            
        var canStatus = RobotController.getCANStatus();
        DogLog.log("CAN/Utilization", canStatus.percentBusUtilization * 100);
        DogLog.log("CAN/TxError", canStatus.txFullCount);
        DogLog.log("CAN/RxError", canStatus.receiveErrorCount);
    }

    private void logDriveStats() {
        if (!LOG_SWERVE_DRIVE) return;

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