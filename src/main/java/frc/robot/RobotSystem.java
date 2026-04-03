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
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.util.ZoneManager.FieldZone;
import frc.robot.util.ZoneManager;

public class RobotSystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;
    private final LEDSubsystem led;
    
    private final Field2d fieldTelemetry = new Field2d();

    //Allows us to disable certain logging for performance reasons
    private final boolean LOG_LIMELIGHTS = true;
    private final boolean LOG_SWERVE_DRIVE = true;
    private final boolean LOG_PDH = true;

    private DrivingMode currentDrivingMode = DrivingMode.NORMAL;
    private DrivingMode lastDrivingMode = DrivingMode.NORMAL;

    private double filteredX, filteredY, filteredTheta, lastFilteredX, lastFilteredY, lastFilteredTheta;

    private final SlewRateLimiter normalXLimiter = new SlewRateLimiter(5.0), normalYLimiter = new SlewRateLimiter(5.0);
    private final SlewRateLimiter normalThetaLimiter = new SlewRateLimiter(10.0);

    private final SlewRateLimiter precisionXLimiter = new SlewRateLimiter(3.0), precisionYLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter precisionThetaLimiter = new SlewRateLimiter(10.0);

    private final AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    private final PowerDistribution pdh = new PowerDistribution(63, ModuleType.kRev);
    private final Notifier pdhNotifier;

    //Pose centered on the front of the hub to reset to if our vision goes haywire
    private final Pose2d ODOMETRY_RESET_POSE = new Pose2d(3.612, 4.027, Rotation2d.kZero);

    private long lastMs = 0;

    public RobotSystem(CommandXboxController driverController, CommandXboxController operatorController) {
        drive = TunerConstants.createDrivetrain();
        intake = new IntakeSubsystem();
        shooter = new ShootingSuperstructure(
            () -> drive.getPose(), 
            () -> drive.getFieldRelativeVelocity(),
            () -> drive.getRotation3d()
        );
        led = new LEDSubsystem(() -> ShiftTracker.hubIsActive());

        //Log the field + robot pose to Elastic
        SmartDashboard.putData("FieldTelemetry", fieldTelemetry);

        ShiftTracker.shiftWarningTrigger.onTrue(led.blink());

        pdhNotifier = new Notifier(() -> {
                logPdhStats();
        });
        pdhNotifier.startPeriodic(0.5);
    }

    public Command forceResetOdometry() {
        return new InstantCommand(() -> drive.resetPose(AllianceUtility.flipPose(ODOMETRY_RESET_POSE)));
    }

    public void setIntakeDefaultCommand(DoubleSupplier rollerSpeed, BooleanSupplier deploy, BooleanSupplier retract, BooleanSupplier agitate) {
        Command intakeDefaultCommand = run(
            () -> {
                double targetRollerSpeed = rollerSpeed.getAsDouble();
                intake.setRollerSpeed(targetRollerSpeed);
            }
        ).beforeStarting(
            () -> {
                Trigger deployTrigger = new Trigger(deploy);
                deployTrigger
                    .onTrue(intake.deployCommand())
                    .onFalse(intake.agitate().onlyIf(agitate));

                Trigger retractTrigger = new Trigger(retract);
                retractTrigger.onTrue(intake.retractCommand());

                Trigger agitateTrigger = new Trigger(agitate);
                agitateTrigger
                    .whileTrue(intake.agitate().repeatedly().onlyIf(deployTrigger.negate()))
                    .onFalse(intake.deployCommand());
            }
        );

        intakeDefaultCommand.addRequirements(intake);
        intake.setDefaultCommand(intakeDefaultCommand);
    }

    public Command dashboardHoodReset() {
        return shooter.dashboardHoodReset();
    }

    public Command shoot() {
        return shooter.shoot();
    }

    public BooleanSupplier needsHopperAgitate() {
        return () -> shooter.needsHopperAgitate();
    }

    public BooleanSupplier isHopperEmpty() {
        return ()-> shooter.isHopperEmpty();
    }

    public void resetAfterAuto() {
        shooter.setState(ShooterState.IDLE);
    }

    public Command agitate() {
        return intake.agitate();
    }

    public Command agitateRepeatedly() {
        return intake.agitate().repeatedly();
    }   

    public Command stopAgitating() {
        return intake.stopCommand();
    }

    public Command clearTransfer() {
        return shooter.clearTransfer();
    }

    public void changeRPMOffset(int delta) {
        shooter.changeRPMOffset(delta);
    }

    public Command changeRpmMap(int delta) {
        return new InstantCommand(() -> shooter.changeRpmMap(delta));
    }

    private void updateShootingState() {
        FieldZone zone = ZoneManager.getZone();

        if (zone.equals(FieldZone.HUB))
            shooter.setState(ShooterState.HUB_TRACKING);
        else if (zone.equals(FieldZone.PASS))
            shooter.setState(ShooterState.PASSING);
        else
            shooter.setState(ShooterState.IDLE);
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
                
                //Square inputs for finer control around zero
                xInput = Math.copySign(xInput * xInput, xInput);
                yInput = Math.copySign(yInput * yInput, yInput);
                thetaInput = Math.copySign(thetaInput * thetaInput, thetaInput);

                if (currentDrivingMode != lastDrivingMode) {
                    normalXLimiter.reset(lastFilteredX);
                    normalYLimiter.reset(lastFilteredY);
                    normalThetaLimiter.reset(lastFilteredTheta);

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
                    filteredX = normalXLimiter.calculate(xInput);
                    filteredY = normalYLimiter.calculate(yInput);
                    filteredTheta = normalThetaLimiter.calculate(thetaInput);
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
            drive.createAutoFactory(),
            intake,
            shooter
        );
    }

    private void updateOdometry() {
        if (Math.abs(drive.getFieldRelativeVelocity().omegaRadiansPerSecond) < LimelightConstants.MAX_VISION_ANGULAR_VELOCITY) {
            double imuAngle = drive.getPose().getRotation().getDegrees();
            PoseEstimate bestPoseEstimate = null;

            for (String limelight : LimelightConstants.LIMELIGHTS) {
                LimelightHelpers.SetRobotOrientation(limelight, imuAngle, 0, 0, 0, 0, 0);

                var poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

                boolean betterEstimate = (bestPoseEstimate == null || isBetterPoseEstimate(poseEstimate, bestPoseEstimate));
                if (isGoodPoseEstimate(poseEstimate) && betterEstimate) {
                    bestPoseEstimate = poseEstimate;
                }
            }

            if (bestPoseEstimate != null) {
                drive.addVisionMeasurement(
                    bestPoseEstimate.pose,
                    bestPoseEstimate.timestampSeconds,
                    LimelightConstants.VISION_STDDEVS
                );
            }
        }
    }

    private boolean isGoodPoseEstimate(PoseEstimate poseEstimate) {
        return
            poseEstimate != null && poseEstimate.pose != null &&
            poseEstimate.tagCount > LimelightConstants.MIN_TAG_COUNT_REJECTION && 
            poseEstimate.avgTagDist < LimelightConstants.MIN_TAG_REJECTION_METERS;
    }

    /**
     * @return Whether the first estimate is better than the second
     */
    private boolean isBetterPoseEstimate(PoseEstimate first, PoseEstimate second) {
        return poseEstimateScore(first) > poseEstimateScore(second);
    }

    /*
     * Weights the different aspects of a pose estimate to give it a comparable score. Tunable to achieve
     * the best pose estimates for our specific purpose.
     */
    private double poseEstimateScore(PoseEstimate p) {
        return
            p.tagCount * LimelightConstants.POSE_ESTIMATE_WEIGHTS[0] +
            Math.min((1.0 / p.avgTagDist), 5) * LimelightConstants.POSE_ESTIMATE_WEIGHTS[2] +
            p.tagSpan * LimelightConstants.POSE_ESTIMATE_WEIGHTS[2];
    }

    public void toggleDisabledLeds(boolean disable) {
        led.setIsDisabled(disable);
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

        //Update the field telemetry's robot pose
        fieldTelemetry.setRobotPose(drivePose);

        DogLog.forceNt.log("Current Zone", ZoneManager.getZone().name());
        DogLog.forceNt.log("Driving Mode", currentDrivingMode.name());

        if (LOG_LIMELIGHTS) {
            for (String ll : LimelightConstants.LIMELIGHTS) {
                PoseEstimate m1Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);
                if (m1Pose != null && !(m1Pose.pose.getX() == 0 && m1Pose.pose.getY() == 0))
                    DogLog.log(ll + "/M1Pose", m1Pose.pose);

                List<Pose3d> visibleTags = new ArrayList<>();
                for (var tag : m1Pose.rawFiducials) {
                    tagFieldLayout.getTagPose(tag.id).ifPresent(visibleTags::add);
                }

                if (!visibleTags.isEmpty())
                    DogLog.log(ll + "/VisibleTags", visibleTags.toArray(new Pose3d[0]));
            }
        }

        long currentMs = System.currentTimeMillis();
        if (currentMs - lastMs >= DogLogUtil.MOTOR_LOGGING_INTERVAL_MS) {
            if (LOG_SWERVE_DRIVE)
                logDriveStats();
            logStats();
            lastMs = currentMs;
        }
    }

    private void logPdhStats() {
        if (LOG_PDH) {
            DogLog.log("PDH/TotalCurrent", pdh.getTotalCurrent());
            DogLog.log("PDH/Voltage", pdh.getVoltage());
            DogLog.log("PDH/Temperature", pdh.getTemperature());
            DogLog.log("PDH/Brownout", pdh.getFaults().Brownout);
            DogLog.log("PDH/HardwareFault", pdh.getFaults().HardwareFault);
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