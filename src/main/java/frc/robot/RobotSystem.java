package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.util.AllianceUtility;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.ShiftTracker;
import frc.robot.util.ZoneManager;

public class RobotSystem extends SubsystemBase {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ShootingSuperstructure shooter;
    private final ClimbSubsystem climb;

    private final Command driverRumble, operatorRumble;

    private final Field2d fieldTelemetry = new Field2d();

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

    public Command shoot() {
        return shooter.shoot()
            .onlyIf(() -> ShiftTracker.canScore() || SmartDashboard.getBoolean("Force Allow Shooting", false));
    }

    private void updateShootingState() {
        if (ZoneManager.inHubZone())
            shooter.setState(ShooterState.HUB_TRACKING);
        else if (ZoneManager.inPassingZone())
            shooter.setState(ShooterState.PASSING);
        else
            shooter.setState(ShooterState.IDLE);
    }

    /**
     * @param x The supplier for driving the robot forward (field centric)
     * @param y The supplier for driving the robot sideways (field centric)
     * @param theta The supplier for rotating the robot
     * @param isFieldCentric Supplier that allows us to toggle between driving modes
     */
    public void setDriveDefaultCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, BooleanSupplier isFieldCentric) {
        drive.setDefaultCommand(
            drive.applyRequest(() -> {
                return isFieldCentric.getAsBoolean() ? 
                    drive.fieldCentric
                        .withVelocityX(-y.getAsDouble() * drive.MAX_SPEED)
                        .withVelocityY(-x.getAsDouble() * drive.MAX_SPEED)
                        .withRotationalRate(-theta.getAsDouble() * drive.MAX_ANGULAR_RATE) :
                    drive.robotCentric
                        .withVelocityX(y.getAsDouble() * drive.MAX_SPEED)
                        .withVelocityY(x.getAsDouble() * drive.MAX_SPEED)
                        .withRotationalRate(-theta.getAsDouble() * drive.MAX_ANGULAR_RATE);
            })
        );
    }

    public Command seedFieldCentric() {
        return runOnce(() -> drive.seedFieldCentric());
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
        LimelightHelpers.SetRobotOrientation("limelight-robot", drive.getPigeon2().getRotation2d().getDegrees(), 0, 0, 0, 0, 0);

        if (AllianceUtility.getAlliance().equals(Alliance.Blue)) {
            PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-robot");
            if (poseEstimate != null && poseEstimate.tagCount > 0) {
                drive.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
            }
        }
        else {
            PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-robot");
            if (poseEstimate != null && poseEstimate.tagCount > 0) {
                drive.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
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
    }
}
