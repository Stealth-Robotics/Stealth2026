package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.ShootingSuperstructure.PassingTarget;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.FieldPose;
import frc.robot.subsystems.LEDSubsystem.DisplayMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSuperstructure.ShooterState;
import frc.robot.util.AllianceUtility;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.ZoneManager.FieldZone;
import frc.robot.util.ShiftTracker;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ZoneManager;

public class RobotSystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;
    private final ClimbSubsystem climb;
    private final LEDSubsystem led;
    
    private boolean isSlowMoActive = false;

    private final Field2d fieldTelemetry = new Field2d();

    private final double MIN_TAG_REJECTION_METERS = 6;
    private final String LOCALIZATION_LIMELIGHT = "limelight-robot";

    private final AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    private final double DRIVE_SHOOT_SLOWDOWN_FACTOR = 0.25;
    private final double DRIVE_USER_SLOWDOWN_FACTOR = 0.3;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(5.0);

    public RobotSystem(CommandXboxController driverController, CommandXboxController operatorController) {
        drive = TunerConstants.createDrivetrain();
        intake = new IntakeSubsystem();
        shooter = new ShootingSuperstructure(() -> drive.getPose(), () -> drive.getFieldRelativeVelocity());
        climb = new ClimbSubsystem();
        led = new LEDSubsystem();

        //Log the field + robot pose to Elastic
        SmartDashboard.putData("FieldTelemetry", fieldTelemetry);

        //Trigger to rumble gamepad when it is okay to shoot into our hub
        Trigger rumbleTrigger = new Trigger(() -> DriverStation.isTeleop() && ShiftTracker.canScore());
        rumbleTrigger
            .onTrue(
                new SequentialCommandGroup(
                    new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1)),
                    new InstantCommand(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 1)),
                    new WaitCommand(0.25),
                    new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)),
                    new InstantCommand(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0))
                )
            );
    }

    public void setIntakeDefaultCommand(DoubleSupplier rollerSpeed, BooleanSupplier deploy) {
        Command intakeDefaultCommand = run(() -> {
            intake.setRollerSpeed(rollerSpeed.getAsDouble());
        }).beforeStarting(() -> {
            Trigger deployToggle = new Trigger(deploy);
            deployToggle.toggleOnTrue(
                new ConditionalCommand(
                    new InstantCommand(() -> intake.retract()),
                    new InstantCommand(() -> intake.deploy()),
                    () -> intake.isDeployed()
                )
            );
        });

        intakeDefaultCommand.addRequirements(intake);
        intake.setDefaultCommand(intakeDefaultCommand);
    }

    public Command setPassingTarget(PassingTarget newTarget) {
        return runOnce(() -> shooter.setPassingTarget(newTarget));
    }

    public Command shoot() {
        return shooter.shoot();     
    }

    public Command agitate() {
        return intake.toss(IntakeSubsystem.INTAKE_TOSS_PERCENTAGE_UP);
    }

    public Command clearTransfer() {
        return shooter.clearTransfer();
    }

    public Command overrideShiftTracker() {
        return shooter.overrideShiftTracker();
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
     * @param isFieldCentric Supplier that allows us to toggle between driving modes
     * 
     * <p>Makes the wheels brake if no gamepad input is provided. Using the isFieldCentric supplier
     * allows us to change modes mid match.</p>
     */
    public void setDriveDefaultCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, BooleanSupplier isFieldCentric, BooleanSupplier brake) {
        drive.setDefaultCommand(
            drive.applyRequest(() -> {
                double filteredX = xLimiter.calculate(x.getAsDouble());
                double filteredY = yLimiter.calculate(y.getAsDouble());
                double filteredTheta = thetaLimiter.calculate(theta.getAsDouble());
                double speed = getDrivingSpeedScaleFactor();

                return brake.getAsBoolean() ?
                drive.brake :

                isFieldCentric.getAsBoolean() ?
                    drive.fieldCentric
                        .withVelocityX(-filteredY * drive.MAX_SPEED * speed)
                        .withVelocityY(-filteredX * drive.MAX_SPEED * speed)
                        .withRotationalRate(-filteredTheta * drive.MAX_ANGULAR_RATE * speed) :
                        
                    drive.robotCentric
                        .withVelocityX(filteredY * drive.MAX_SPEED * speed)
                        .withVelocityY(filteredX * drive.MAX_SPEED * speed)
                        .withRotationalRate(-filteredTheta * drive.MAX_ANGULAR_RATE * speed);
            })
        );
    }

    /**
     * Allows us to slow down when performing certain actions like shooting or climbing
     */
    public double getDrivingSpeedScaleFactor() {
        if (shooter.isShooting()) {
            return DRIVE_SHOOT_SLOWDOWN_FACTOR;
        } else if (this.isSlowMoActive) {
            return DRIVE_USER_SLOWDOWN_FACTOR;
        }

        return 1.0;
    }

    public Command driveToPose(FieldPose targetPose) {
        return drive.goToPose(() -> targetPose);
    }

    public Command activateSlowMo() {
        return new StartEndCommand(
            () -> {
                isSlowMoActive = true;
            },
            () -> {
                isSlowMoActive = false;
            }
        );
    }

    /*
     * Macro that rotates the robot into the turret's operating range (plus a little extra for tolerance)
     */
    public Command rotateRobotToShoot() {
        return drive.rotateToAngle(() -> drive.getPose().getRotation()
            .plus(Rotation2d.fromDegrees(shooter.turretLockError + (5 * Math.signum(shooter.turretLockError)))));
    }

    public Command seedFieldCentric() {
         return runOnce(() -> drive.seedFieldCentric()).andThen(() -> shooter.setState(ShooterState.HUB_TRACKING));

        // return new ConditionalCommand(
        //         runOnce(()-> drive.resetPose(new Pose2d(3,4,Rotation2d.fromDegrees(0)))), 
        //         runOnce(() -> drive.resetPose(new Pose2d(14,4,Rotation2d.fromDegrees(180)))),
        //         () -> AllianceUtility.getAlliance().equals(Alliance.Blue))
        //     .andThen(runOnce(() -> shooter.setState(ShooterState.HUB_TRACKING)))
        //     .andThen(runOnce(() -> drive.seedFieldCentric()));
    }

    public Autos getAutos() {
        return new Autos(
            drive.createAutoFactory(),
            drive,
            intake,
            shooter,
            climb
        );
    }

    private void updateOdometryEstimateWithLimelight() {
        double imuAngle = drive.getPose().getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(LOCALIZATION_LIMELIGHT, imuAngle, 0, 0, 0, 0, 0);

        double robotAngularVelocity = drive.getFieldRelativeVelocity().omegaRadiansPerSecond;
        if (Math.abs(robotAngularVelocity) < Math.PI) {
            PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LOCALIZATION_LIMELIGHT);
            if (poseEstimate != null && poseEstimate.tagCount > 0 && poseEstimate.avgTagDist < MIN_TAG_REJECTION_METERS) {
                drive.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, VecBuilder.fill(.7, .7, 99999));
                ShotCalculator.updateVisionLatency(poseEstimate.latency);
            }
        }

        //Advantagescope logging
        var currentTags = LimelightHelpers.getLatestResults(LOCALIZATION_LIMELIGHT).targets_Fiducials;
        
        Pose3d[] visibleTags = new Pose3d[currentTags.length];
        for (int i = 0; i < currentTags.length; i++) {
            Pose3d tagPose = tagFieldLayout.getTagPose((int) currentTags[i].fiducialID).orElse(null);
            if (tagPose != null)
                visibleTags[i] = tagPose;
        }

        DogLog.log("VisibleTagPoses", visibleTags);
    }

    public Command homeClimber() { return climb.stow(); }
    public Command toggleClimb() { return climb.toggleClimb(); }
    public void disabledLeds() {
        led.changeDisplayMode(DisplayMode.DISABLED);
    }

    @Override
    public void periodic() {
        ZoneManager.updateRobotPositionAndVelocity(drive.getPose());

        updateShootingState();
        updateOdometryEstimateWithLimelight();

        //Update the field telemetry's robot pose
        fieldTelemetry.setRobotPose(drive.getPose());

        DogLog.log("Current Zone", ZoneManager.getZone().name());

        //TODO: Drive telemetry for testing
        DogLog.log("Drive/ChassisSpeeds", drive.getRobotRelativeVelocity());
        DogLog.log("Drive/ModuleStates", drive.getModuleStates());
        DogLog.log("Drive/Rotation", drive.getPose().getRotation());
        // DogLog.log("Drive/SysId/ActiveRoutine", drive.getSysIdRoutineName());
    }
}