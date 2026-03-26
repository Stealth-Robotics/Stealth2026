package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.FieldPose;
import frc.robot.subsystems.LEDSubsystem.DisplayMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSuperstructure.ShooterState;
import frc.robot.util.DogLogUtil;
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

    private final boolean LOG_LIMELIGHTS = true;
    private final boolean LOG_SWERVE_DRIVE = true;

    private final String FRONT_LL = "limelight-front";
    private final String RIGHT_LL = "limelight-right";

    private final Vector<N3> VISION_STDDEVS = VecBuilder.fill(0.2, 0.2, 99999);

    private final double MAX_VISION_ANGULAR_VELOCITY = Math.PI; //Rad/s

    private final double MIN_TAG_REJECTION_METERS = 10;

    private DrivingMode currentDrivingMode = DrivingMode.NORMAL;

    //Throttle status refreshes to reduce CAN bus usage
    private long lastStatusRefreshMs = 0;

    public enum DrivingMode {
        NORMAL(1.0),
        PRECISION(0.75);

        /**
         * Allows us to slow down when performing certain actions like shooting or climbing
         */
        final double slowingFactor;

        DrivingMode(double slowingFactor) {
            this.slowingFactor = slowingFactor;
        }

        double getSlowingFactor() {
            return slowingFactor;
        }
    }

    private final SlewRateLimiter normalXLimiter = new SlewRateLimiter(5.0);
    private final SlewRateLimiter normalYLimiter = new SlewRateLimiter(5.0);
    private final SlewRateLimiter normalThetaLimiter = new SlewRateLimiter(10.0);

    private final SlewRateLimiter precisionXLimiter = new SlewRateLimiter(2.0);
    private final SlewRateLimiter precisionYLimiter = new SlewRateLimiter(2.0);
    private final SlewRateLimiter precisionThetaLimiter = new SlewRateLimiter(5.0);

    public RobotSystem(CommandXboxController driverController, CommandXboxController operatorController) {
        drive = TunerConstants.createDrivetrain();
        intake = new IntakeSubsystem();
        shooter = new ShootingSuperstructure(() -> drive.getPose(), () -> drive.getFieldRelativeVelocity());
        led = new LEDSubsystem();

        //Log the field + robot pose to Elastic
        SmartDashboard.putData("FieldTelemetry", fieldTelemetry);

        //Rumble shift warning
        ShiftTracker.shiftWarningTrigger.onTrue(
            new StartEndCommand(
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0),
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)
            ).withTimeout(1)
        );

        ShiftTracker.shiftWarningTrigger.onTrue(led.blink());
    }

    public void setIntakeDefaultCommand(DoubleSupplier rollerSpeed, BooleanSupplier deploy, BooleanSupplier retract) {
        Command intakeDefaultCommand = run(
            () -> {
                double targetRollerSpeed = rollerSpeed.getAsDouble();
                intake.setRollerSpeed(targetRollerSpeed);

                if (Math.abs(targetRollerSpeed) > 0.1) intake.isIntaking(true);
                else intake.isIntaking(false);
            }
        ).beforeStarting(
            () -> {
                Trigger deployTrigger = new Trigger(deploy);
                deployTrigger.onTrue(intake.deployCommand());

                Trigger retractTrigger = new Trigger(retract);
                retractTrigger.onTrue(intake.retractCommand());
            }
        );

        intakeDefaultCommand.addRequirements(intake);
        intake.setDefaultCommand(intakeDefaultCommand);
    }

    public Command shoot() {
        return shooter.shoot()
            .beforeStarting(() -> currentDrivingMode = DrivingMode.PRECISION)
            .finallyDo(() -> currentDrivingMode = DrivingMode.NORMAL);
    }

    public void resetAfterAuto() {
        shooter.setState(ShooterState.IDLE);
    }

    public Command agitate() {
        return intake.agitate();
    }

    public Command clearTransfer() {
        return shooter.clearTransfer();
    }

    public void changeRPMOffset(int delta) {
        shooter.changeRPMOffset(delta);
    }

    private void updateShootingState() {
        FieldZone zone = ZoneManager.getZone();

        if (zone.equals(FieldZone.HUB))
            shooter.setState(ShooterState.HUB_TRACKING);
        else if (zone.equals(FieldZone.PASS))
            shooter.setState(ShooterState.PASSING);
        else if (zone.equals(FieldZone.TRENCH))
            shooter.setState(ShooterState.TRENCH);
        else
            shooter.setState(ShooterState.IDLE);
    }

    /**
     * @param x The supplier for driving the robot forward (field centric)
     * @param y The supplier for driving the robot sideways (field centric)
     * @param theta The supplier for rotating the robot
     * @param isFieldCentric Supplier that allows us to toggle between driving modes
     * 
     * <p>Makes the wheels brake if no gamepad input is provided. Using the isFieldCentric supplier
     * allows us to change modes mid match.</p>
     */
    public void setDriveDefaultCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        drive.setDefaultCommand(
            drive.applyRequest(() -> {
                double filteredX, filteredY, filteredTheta;

                if (currentDrivingMode.equals(DrivingMode.PRECISION)) {
                    filteredX = precisionXLimiter.calculate(x.getAsDouble());
                    filteredY = precisionYLimiter.calculate(y.getAsDouble());
                    filteredTheta = precisionThetaLimiter.calculate(theta.getAsDouble());

                    //Reset other slew rate limiters
                    normalXLimiter.reset(0);
                    normalYLimiter.reset(0);
                    normalThetaLimiter.reset(0);
                }
                else {
                    filteredX = normalXLimiter.calculate(x.getAsDouble());
                    filteredY = normalYLimiter.calculate(y.getAsDouble());
                    filteredTheta = normalThetaLimiter.calculate(theta.getAsDouble());

                    //Reset other slew rate limiters
                    precisionXLimiter.reset(0);
                    precisionYLimiter.reset(0);
                    precisionThetaLimiter.reset(0);
                }

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
        double imuAngle = drive.getPose().getRotation().getDegrees();

        LimelightHelpers.SetRobotOrientation(FRONT_LL, imuAngle, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(RIGHT_LL, imuAngle, 0, 0, 0, 0, 0);

        if (Math.abs(drive.getFieldRelativeVelocity().omegaRadiansPerSecond) < MAX_VISION_ANGULAR_VELOCITY) {
            var frontPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(FRONT_LL);
            var rightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RIGHT_LL);

            if (isGoodPoseEstimate(frontPoseEstimate))
                drive.addVisionMeasurement(
                    frontPoseEstimate.pose, 
                    frontPoseEstimate.timestampSeconds, 
                    VISION_STDDEVS
                );
            else if (isGoodPoseEstimate(rightPoseEstimate))
                drive.addVisionMeasurement(
                    rightPoseEstimate.pose, 
                    rightPoseEstimate.timestampSeconds, 
                    VISION_STDDEVS
                );
        }
    }

    private boolean isGoodPoseEstimate(PoseEstimate poseEstimate) {
        return poseEstimate != null && poseEstimate.tagCount > 0 && poseEstimate.avgTagDist < MIN_TAG_REJECTION_METERS;
    }

    public void setLEDMode(DisplayMode ledDisplayMode) {
        led.changeDisplayMode(ledDisplayMode);
    }

    public void resetFuelShotCount() {
        shooter.resetFuelShotCount();
    }

    @Override
    public void periodic() {
        ZoneManager.updateRobotPose(drive.getPose());

        updateShootingState();

        //Update odometry with our Limelight's and also log everything
        updateOdometry();

        //Update the field telemetry's robot pose
        fieldTelemetry.setRobotPose(drive.getPose());

        //Update LED state
        if (ShiftTracker.hubIsActive())
            led.changeDisplayMode(DisplayMode.HUB_ACTIVE);
        else 
            led.changeDisplayMode(DisplayMode.HUB_INACTIVE);

        DogLog.log("Current Zone", ZoneManager.getZone().name());
        DogLog.log("Driving Mode", currentDrivingMode.name());

        if (LOG_SWERVE_DRIVE) logDriveStats();

        if (LOG_LIMELIGHTS) {
             //Log the megatag 1 poses of our limelights
            LimelightHelpers.LimelightResults frontLLResults = LimelightHelpers.getLatestResults(FRONT_LL);
            LimelightHelpers.LimelightResults rightLLResults = LimelightHelpers.getLatestResults(RIGHT_LL);

            if (frontLLResults != null) {
                var m1Pose = frontLLResults.getBotPose3d_wpiBlue();
                if (m1Pose != null) {
                    DogLog.log(FRONT_LL + "/wpiBlue_Pose3d", m1Pose);
                }
            }

            if (rightLLResults != null) {
                var m1Pose = rightLLResults.getBotPose3d_wpiBlue();
                if (m1Pose != null) {
                    DogLog.log(RIGHT_LL + "/wpiBlue_Pose3d", m1Pose);
                }
            }
        }
    }

    private void logDriveStats() {
        // Throttle logging of this data. Note that swerve updates these values to calling refresh false is correct.
        long nowMs = System.currentTimeMillis();
        if (nowMs - lastStatusRefreshMs >= DogLogUtil.MOTOR_LOGGING_INTERVAL_MS) {
            DogLog.log("Drive/ChassisSpeeds", drive.getRobotRelativeVelocity());
            DogLog.log("Drive/ModuleStates", drive.getModuleStates());
            DogLog.log("Drive/Rotation", drive.getPose().getRotation());

            for (var module : drive.getModules()) {
                DogLogUtil.logDouble("Drive/" + TunerConstants.getDeviceName(module.getDriveMotor().getDeviceID()) + "_Current",
                    module.getDriveMotor().getSupplyCurrent(false).getValueAsDouble());
                DogLogUtil.logDouble("Drive/" + TunerConstants.getDeviceName(module.getSteerMotor().getDeviceID()) + "_Current",
                    module.getSteerMotor().getSupplyCurrent(false).getValueAsDouble());
                DogLogUtil.logDouble("Drive/" + TunerConstants.getDeviceName(module.getDriveMotor().getDeviceID()) + "_Temperature_C",
                    module.getDriveMotor().getDeviceTemp(false).getValueAsDouble());
                DogLogUtil.logDouble("Drive/" + TunerConstants.getDeviceName(module.getSteerMotor().getDeviceID()) + "_Temperature_C",
                    module.getSteerMotor().getDeviceTemp(false).getValueAsDouble());
            }

            lastStatusRefreshMs = nowMs;
        }        
    }
}