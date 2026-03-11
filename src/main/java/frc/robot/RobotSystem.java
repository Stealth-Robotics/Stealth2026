package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSuperstructure.ShooterState;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.ZoneManager.FieldZone;
import frc.robot.util.ShiftTracker;
import frc.robot.util.ShotTrajectoryCalculator;
import frc.robot.util.ZoneManager;

public class RobotSystem extends SubsystemBase {
    
    private double maxDriveSpeed = 1f;
    private double maxAngularRate = 1f;
    
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;
    private final ClimbSubsystem climb;

    private final Command driverRumble, operatorRumble;

    private final Field2d fieldTelemetry = new Field2d();

    // When true we slow the robot for precision control while operator holds the button
    private boolean isSlowMoActive = false;
    private boolean isIntakeManuallyDeployed = false;

    private final static double MIN_TAG_REJECTION_METERS = 4;
    // TODO: Needs tuning, currently just a guess
    private final static double MAX_DRIVE_SHOOT_MODIFIER = 0.25f;
    private final static double MAX_ANGLE_SHOOT_MODIFIER = 0.25f;
    private final static double MAX_SLOWMO_MODIFIER = 0.1f;
    private final static double MAX_ROLLER_SPEED = 0.75f;

    public RobotSystem(Command driverRumble, Command operatorRumble) {
        drive = TunerConstants.createDrivetrain();
        intake = new IntakeSubsystem();
        shooter = new ShootingSuperstructure(() -> drive.getPose(), () -> drive.getFieldRelativeVelocity());
        climb = new ClimbSubsystem();

        this.driverRumble = driverRumble;
        this.operatorRumble = operatorRumble;

        this.maxDriveSpeed = drive.MAX_SPEED;
        this.maxAngularRate = drive.MAX_ANGULAR_RATE;

        //Log the field + robot pose to Elastic
        SmartDashboard.putData("FieldTelemetry", fieldTelemetry);
    }

    public void setIntakeDefaultCommand(DoubleSupplier rollerSpeed, BooleanSupplier deploy) {
        Command intakeDefaultCommand = run(() -> {
            if (deploy.getAsBoolean()) {
                intake.deploy();
                intake.setRollerSpeed(rollerSpeed.getAsDouble() * MAX_ROLLER_SPEED);
            }
            else {
                if (rollerSpeed.getAsDouble() < -0.01) {
                    intake.setRollerSpeed(rollerSpeed.getAsDouble() * MAX_ROLLER_SPEED);
                }
                else intake.stop();
                
                if (!isIntakeManuallyDeployed){
                    intake.retract();
                }
            }
        });

        intakeDefaultCommand.addRequirements(intake);
        intake.setDefaultCommand(intakeDefaultCommand);
    }

    public Command shoot() {
        return shooter.shoot()
            .onlyIf(() -> ShiftTracker.canScore() || SmartDashboard.getBoolean("Force Allow Shooting", false));
    }

    public Command clearTransfer() {
        return shooter.clearTransfer();
    }

    private void updateShootingState() {
        FieldZone zone = ZoneManager.getZone();

        if (zone.equals(FieldZone.HUB))
            shooter.setState(ShooterState.HUB_TRACKING);
        else if (zone.equals(FieldZone.LEFT_PASS) || zone.equals(FieldZone.RIGHT_PASS))
            shooter.setState(ShooterState.PASSING);
        else
            shooter.setState(ShooterState.IDLE);
    }

    public Command toggleIntake() {
        return runOnce(() -> {
            isIntakeManuallyDeployed = !isIntakeManuallyDeployed;
            if (isIntakeManuallyDeployed) {
                intake.deploy();
            }
            else {
                intake.retract();
            }
        });
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
    public void setDriveDefaultCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, BooleanSupplier isFieldCentric) {
        drive.setDefaultCommand(
            drive.applyRequest(() -> {
                return (Math.abs(x.getAsDouble() + y.getAsDouble() + theta.getAsDouble()) > 0) ?
                    isFieldCentric.getAsBoolean() ?
                        drive.fieldCentric
                            .withVelocityX(-y.getAsDouble() * maxDriveSpeed)
                            .withVelocityY(-x.getAsDouble() * maxDriveSpeed)
                            .withRotationalRate(-theta.getAsDouble() * maxAngularRate) :
                        drive.robotCentric
                            .withVelocityX(y.getAsDouble() * maxDriveSpeed)
                            .withVelocityY(x.getAsDouble() * maxDriveSpeed)
                            .withRotationalRate(-theta.getAsDouble() * maxAngularRate) :
                    drive.brake;
            })
        );
    }

    //TODO: Temporary method for testing
    public Command driveToClimb() {
        return drive.goToPose(() -> new Pose2d(14.922, 3.891, Rotation2d.kZero));
    }

    public Command rotateRobotToShoot() {
        return drive.goToPose(() -> drive.getPose().rotateBy(Rotation2d.fromDegrees(shooter.turretLockError)));
    }

    public Command seedFieldCentric() {
        return runOnce(() -> drive.seedFieldCentric()).andThen(() -> shooter.setState(ShooterState.IDLE));
    }

    public Command slowDown() {
        return new StartEndCommand(
            () -> isSlowMoActive = true,
            () -> isSlowMoActive = false,
            this
        );
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
        LimelightHelpers.SetRobotOrientation("limelight-robot", imuAngle, 0, 0, 0, 0, 0);

        double robotAngularVelocity = drive.getFieldRelativeVelocity().omegaRadiansPerSecond;
        if (Math.abs(robotAngularVelocity) < Math.PI) {
            PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-robot");
            if (poseEstimate != null && poseEstimate.tagCount > 0 && poseEstimate.avgTagDist < MIN_TAG_REJECTION_METERS) {
                //TODO: Tune standard deviations
                drive.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
                ShotTrajectoryCalculator.updateVisionLatency(poseEstimate.latency);
            }
        }
    }

    @Override
    public void periodic() {
        ZoneManager.updateWithRobotPose(drive.getPose());

        updateShootingState();
        updateOdometryEstimateWithLimelight();

        //Update the field telemetry's robot pose
        fieldTelemetry.setRobotPose(drive.getPose());
    
        // Slow mode takes precedence over shooter speed modifiers
        if (isSlowMoActive) {
            this.maxDriveSpeed = drive.MAX_SPEED * MAX_SLOWMO_MODIFIER;
            this.maxAngularRate = drive.MAX_ANGULAR_RATE * MAX_SLOWMO_MODIFIER;
        }
        else if (shooter.isActiveShooter()) {
            this.maxAngularRate = drive.MAX_ANGULAR_RATE * MAX_ANGLE_SHOOT_MODIFIER;
            this.maxDriveSpeed = drive.MAX_SPEED * MAX_DRIVE_SHOOT_MODIFIER;
        }
        else {
            this.maxAngularRate = drive.MAX_ANGULAR_RATE;
            this.maxDriveSpeed = drive.MAX_SPEED;
        }

        DogLog.log("Current Zone", ZoneManager.getZone().name());
        DogLog.log("Turret Lock Error", shooter.turretLockError);
    }
}
