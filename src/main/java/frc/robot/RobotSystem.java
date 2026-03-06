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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSuperstructure.ShooterState;
import frc.robot.util.Elastic;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.ZoneManager.FieldZone;
import frc.robot.util.ShiftTracker;
import frc.robot.util.ShotTrajectoryCalculator;
import frc.robot.util.ZoneManager;

public class RobotSystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;
    private final ClimbSubsystem climb;

    private final Command driverRumble, operatorRumble;

    private final Field2d fieldTelemetry = new Field2d();

    private final static double MIN_TAG_REJECTION_METERS = 4;

    public RobotSystem(Command driverRumble, Command operatorRumble) {
        drive = TunerConstants.createDrivetrain();
        intake = new IntakeSubsystem();
        shooter = new ShootingSuperstructure(() -> drive.getPose(), () -> drive.getFieldRelativeVelocity());
        climb = new ClimbSubsystem();

        this.driverRumble = driverRumble;
        this.operatorRumble = operatorRumble;

        //Log the field + robot pose to Elastic
        SmartDashboard.putData("FieldTelemetry", fieldTelemetry);
    }

    public void setIntakeDefaultCommand(DoubleSupplier rollerSpeed, BooleanSupplier deploy) {
        Command intakeDefaultCommand = run(() -> {
            if (deploy.getAsBoolean()) {
                intake.deploy();
                intake.setRollerSpeed(rollerSpeed.getAsDouble() * 0.8);
            }
            else {
                if (rollerSpeed.getAsDouble() < -0.01) {
                    intake.setRollerSpeed(rollerSpeed.getAsDouble() * 0.8);
                }
                else intake.stop();
                
                intake.retract();
            }
        });

        intakeDefaultCommand.addRequirements(intake);
        intake.setDefaultCommand(intakeDefaultCommand);
    }

    public Command shoot() {
        return shooter.shoot();
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
                            .withVelocityX(-y.getAsDouble() * drive.MAX_SPEED)
                            .withVelocityY(-x.getAsDouble() * drive.MAX_SPEED)
                            .withRotationalRate(-theta.getAsDouble() * drive.MAX_ANGULAR_RATE) :
                        drive.robotCentric
                            .withVelocityX(y.getAsDouble() * drive.MAX_SPEED)
                            .withVelocityY(x.getAsDouble() * drive.MAX_SPEED)
                            .withRotationalRate(-theta.getAsDouble() * drive.MAX_ANGULAR_RATE) :
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

        DogLog.log("Current Zone", ZoneManager.getZone().name());
        DogLog.log("Turret Lock Error", shooter.turretLockError);
    }
}
