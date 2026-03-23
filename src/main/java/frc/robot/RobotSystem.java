package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.ZoneManager.FieldZone;
import frc.robot.util.ZoneManager;

public class RobotSystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;
    private final LEDSubsystem led;

    private final Field2d fieldTelemetry = new Field2d();

    private final double MIN_TAG_REJECTION_METERS = 6;
    private final String LOCALIZATION_LIMELIGHT = "limelight-robot";

    private final AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltAndymark);

    private DrivingMode currentDrivingMode = DrivingMode.NORMAL;

    // Throttle status refreshes to reduce CAN bus usage
    private long lastStatusRefreshMs = 0;

    public enum DrivingMode {
        NORMAL(1.0),
        SHOOTING(0.75),
        PRECISION(0.75),
        ROBOT_CENTRIC(1.0);

        /**
         * Allows us to slow down when performing certain actions like shooting or
         * climbing
         */
        final double slowingFactor;

        DrivingMode(double slowingFactor) {
            this.slowingFactor = slowingFactor;
        }

        double getSlowingFactor() {
            return slowingFactor;
        }
    }

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(5.0);

    public RobotSystem(CommandXboxController driverController, CommandXboxController operatorController) {
        drive = TunerConstants.createDrivetrain();
        intake = new IntakeSubsystem();
        shooter = new ShootingSuperstructure(() -> drive.getPose(), () -> drive.getFieldRelativeVelocity());
        led = new LEDSubsystem();

        // Log the field + robot pose to Elastic
        SmartDashboard.putData("FieldTelemetry", fieldTelemetry);
    }

    public void setIntakeDefaultCommand(DoubleSupplier rollerSpeed, BooleanSupplier deploy, BooleanSupplier retract) {
        Command intakeDefaultCommand = run(
                () -> {
                    double targetRollerSpeed = rollerSpeed.getAsDouble();
                    intake.setRollerSpeed(targetRollerSpeed);

                    if (Math.abs(targetRollerSpeed) > 0.2)
                        intake.isIntaking(true);
                    else
                        intake.isIntaking(false);
                }).beforeStarting(
                        () -> {
                            Trigger deployTrigger = new Trigger(deploy);
                            deployTrigger.onTrue(intake.deployCommand());

                            Trigger retractTrigger = new Trigger(retract);
                            retractTrigger.onTrue(intake.retractCommand());
                        });

        intakeDefaultCommand.addRequirements(intake);
        intake.setDefaultCommand(intakeDefaultCommand);
    }

    public Command shoot() {
        return shooter.shoot()
                // .alongWith(intake.agitate().repeatedly())
                .beforeStarting(() -> {
                    currentDrivingMode = DrivingMode.SHOOTING;
                })
                .finallyDo(() -> {
                    currentDrivingMode = DrivingMode.NORMAL;
                });
    }

    public void resetAfterAuto() {
        CommandScheduler.getInstance().requiring(intake).cancel();
        CommandScheduler.getInstance().requiring(shooter).cancel();
        shooter.setState(ShooterState.IDLE);
    }

    public Command agitate() {
        return intake.agitate();
    }

    public Command clearTransfer() {
        return shooter.clearTransfer();
    }

    public Command intakeDeploy() {
        return intake.deployCommand();
    }

    public Command intakeRetract() {
        return intake.retractCommand();
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
     * @param x              The supplier for driving the robot forward (field
     *                       centric)
     * @param y              The supplier for driving the robot sideways (field
     *                       centric)
     * @param theta          The supplier for rotating the robot
     * @param isFieldCentric Supplier that allows us to toggle between driving modes
     * 
     *                       <p>
     *                       Makes the wheels brake if no gamepad input is provided.
     *                       Using the isFieldCentric supplier
     *                       allows us to change modes mid match.
     *                       </p>
     */
    public void setDriveDefaultCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        drive.setDefaultCommand(
                drive.applyRequest(() -> {
                    double filteredX = xLimiter.calculate(x.getAsDouble());
                    double filteredY = yLimiter.calculate(y.getAsDouble());
                    double filteredTheta = thetaLimiter.calculate(theta.getAsDouble());
                    double speed = currentDrivingMode.getSlowingFactor();

                    return (currentDrivingMode.equals(DrivingMode.ROBOT_CENTRIC)) ? drive.robotCentric
                            .withVelocityX(filteredY * drive.MAX_SPEED * speed)
                            .withVelocityY(filteredX * drive.MAX_SPEED * speed)
                            .withRotationalRate(-filteredTheta * drive.MAX_ANGULAR_RATE * speed)
                            : drive.fieldCentric
                                    .withVelocityX(-filteredY * drive.MAX_SPEED * speed)
                                    .withVelocityY(-filteredX * drive.MAX_SPEED * speed)
                                    .withRotationalRate(-filteredTheta * drive.MAX_ANGULAR_RATE * speed);
                }));
    }

    public void toggleDrivingMode() {
        if (currentDrivingMode.equals(DrivingMode.ROBOT_CENTRIC))
            currentDrivingMode = DrivingMode.NORMAL;
        else
            currentDrivingMode = DrivingMode.ROBOT_CENTRIC;
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
                });
    }

    public Command keepRobotLockedWithTurret() {
        return new RepeatCommand(
                new ConditionalCommand(
                        drive.rotateToAngle(() -> drive.getPose().getRotation()
                                .plus(Rotation2d.fromDegrees(
                                        shooter.turretLockError + (5 * Math.signum(shooter.turretLockError))))),
                        new InstantCommand(),
                        () -> Math.abs(shooter.turretLockError) > 0));
    }

    public Command seedFieldCentric() {
        return runOnce(() -> drive.seedFieldCentric());
    }

    public Autos getAutos() {
        return new Autos(
                drive.createAutoFactory(),
                intake,
                shooter);
    }

    // TODO: actually utilize these, need to figure out logic for activation
    private void setIMUMode_Waiting(String ll_name) {
        LimelightHelpers.SetIMUMode(ll_name, 1);
    }

    private void setIMUMode_Active(String ll_name) {
        LimelightHelpers.SetIMUMode(ll_name, 4);
    }

    private void updateOdometry() {
        double imuAngle = drive.getPose().getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(LOCALIZATION_LIMELIGHT, imuAngle, 0, 0, 0, 0, 0);

        double robotAngularVelocity = drive.getFieldRelativeVelocity().omegaRadiansPerSecond;
        if (Math.abs(robotAngularVelocity) < Math.PI) {
            PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LOCALIZATION_LIMELIGHT);

            if (poseEstimate != null && poseEstimate.tagCount > 0
                    && poseEstimate.avgTagDist < MIN_TAG_REJECTION_METERS) {
                drive.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds,
                        VecBuilder.fill(.7, .7, 99999));
            }
        }
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

        // Update odometry with our Limelight's and also log everything
        updateOdometry();

        // Update the field telemetry's robot pose
        fieldTelemetry.setRobotPose(drive.getPose());

        DogLog.log("Current Zone", ZoneManager.getZone().name());
        DogLog.log("Driving Mode", currentDrivingMode.name());

        this.logDriveStats();

        LimelightHelpers.LimelightResults llResults = LimelightHelpers.getLatestResults(LOCALIZATION_LIMELIGHT);
        if (llResults != null) {
            // Log Limelight hardware temperature (if available)
            if (llResults.hardware != null) {
                DogLogUtil.logDouble(LOCALIZATION_LIMELIGHT + "/Hardware_Temperature_C",
                        llResults.hardware.temperature);
                DogLogUtil.logDouble(LOCALIZATION_LIMELIGHT + "/Capture_Latency", llResults.latency_capture);
            }

            // Log visible tag poses
            var currentTags = llResults.targets_Fiducials;
            if (currentTags != null && currentTags.length > 0) {
                Pose3d[] visibleTags = new Pose3d[currentTags.length];
                for (int i = 0; i < currentTags.length; i++) {
                    Pose3d tagPose = tagFieldLayout.getTagPose((int) currentTags[i].fiducialID).orElse(null);
                    if (tagPose != null)
                        visibleTags[i] = tagPose;
                }

                DogLog.log(LOCALIZATION_LIMELIGHT + "/VisibleTagPoses", visibleTags);
            }

            // Log M1 Pose data
            var m1Pose = llResults.getBotPose3d_wpiBlue();
            if (m1Pose != null) {
                DogLog.log(LOCALIZATION_LIMELIGHT + "/wpiBlue_Pose3d", m1Pose);
            }
        }

    }

    private void logDriveStats() {

        // Throttle logging of this data. Note that swerve updates these values to
        // calling refresh false is correct.
        long nowMs = System.currentTimeMillis();
        if (nowMs - lastStatusRefreshMs >= DogLogUtil.MOTOR_LOGGING_INTERVAL_MS) {
            DogLog.log("Drive/ChassisSpeeds", drive.getRobotRelativeVelocity());
            DogLog.log("Drive/ModuleStates", drive.getModuleStates());
            DogLog.log("Drive/Rotation", drive.getPose().getRotation());

            for (var module : drive.getModules()) {
                DogLogUtil.logDouble(
                        "Drive/" + TunerConstants.getDeviceName(module.getDriveMotor().getDeviceID()) + "_Current",
                        module.getDriveMotor().getSupplyCurrent(false).getValueAsDouble());
                DogLogUtil.logDouble(
                        "Drive/" + TunerConstants.getDeviceName(module.getSteerMotor().getDeviceID()) + "_Current",
                        module.getSteerMotor().getSupplyCurrent(false).getValueAsDouble());
                DogLogUtil.logDouble(
                        "Drive/" + TunerConstants.getDeviceName(module.getDriveMotor().getDeviceID())
                                + "_Temperature_C",
                        module.getDriveMotor().getDeviceTemp(false).getValueAsDouble());
                DogLogUtil.logDouble(
                        "Drive/" + TunerConstants.getDeviceName(module.getSteerMotor().getDeviceID())
                                + "_Temperature_C",
                        module.getSteerMotor().getDeviceTemp(false).getValueAsDouble());
            }

            lastStatusRefreshMs = nowMs;
        }
    }
}
