package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShootingSuperstructure;
import frc.robot.subsystems.ShootingSuperstructure.ShooterState;
import frc.robot.util.DogLogUtil;
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
    private final LEDSubsystem led;

    private final CommandXboxController driverController, operatorController;

    private final Field2d fieldTelemetry = new Field2d();

    private final double MIN_TAG_REJECTION_METERS = 4;
    private final String LOCALIZATION_LIMELIGHT = "limelight-robot";

    public RobotSystem(CommandXboxController driverController, CommandXboxController operatorController) {
        drive = TunerConstants.createDrivetrain();
        intake = new IntakeSubsystem();
        shooter = new ShootingSuperstructure(() -> drive.getPose(), () -> drive.getFieldRelativeVelocity());
        climb = new ClimbSubsystem();
        led = new LEDSubsystem();

        this.driverController = driverController;
        this.operatorController = operatorController;

        //Log the field + robot pose to Elastic
        SmartDashboard.putData("FieldTelemetry", fieldTelemetry);

        //Trigger to rumble gamepad when it is okay to shoot into our hub
        var rumbleTrigger = new Trigger(() -> DriverStation.isTeleop() && ShiftTracker.canScore());
            // .onTrue(
            //     new SequentialCommandGroup(
            //         new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5)),
            //         new WaitCommand(200),
            //         new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5)),
            //         new WaitCommand(200),
            //         new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5)),
            //         new WaitCommand(200),
            //         new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5))
            //     )
            // );
    }

    public void setIntakeDefaultCommand(DoubleSupplier rollerSpeed, BooleanSupplier deploy) {
        final double INTAKE_SPEED = 0.75;

        Command intakeDefaultCommand = run(() -> {
            if (deploy.getAsBoolean()) {
                intake.deploy();
                intake.setRollerSpeed(INTAKE_SPEED);
            }
            else {
                if (rollerSpeed.getAsDouble() < -0.01) {
                    intake.setRollerSpeed(-INTAKE_SPEED);
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

    //TODO: Temporary method for testing
    public Command driveToClimb() {
        return drive.goToPose(() -> new Pose2d(14.922, 3.891, Rotation2d.kZero));
    }

    /*
     * Macro that rotates the robot into the turret's operating range (plus a little extra for tolerance)
     */
    public Command rotateRobotToShoot() {
        return drive.rotateToAngle(() -> drive.getPose().getRotation()
            .plus(Rotation2d.fromDegrees(shooter.turretLockError + (5 * Math.signum(shooter.turretLockError)))));
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
        LimelightHelpers.SetRobotOrientation(LOCALIZATION_LIMELIGHT, imuAngle, 0, 0, 0, 0, 0);

        double robotAngularVelocity = drive.getFieldRelativeVelocity().omegaRadiansPerSecond;
        if (Math.abs(robotAngularVelocity) < Math.PI) {
            PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LOCALIZATION_LIMELIGHT);
            if (poseEstimate != null && poseEstimate.tagCount > 0 && poseEstimate.avgTagDist < MIN_TAG_REJECTION_METERS) {
                //TODO: Tune standard deviations
                drive.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
                ShotTrajectoryCalculator.updateVisionLatency(poseEstimate.latency);
            }
        }
    }

    @Override
    public void periodic() {
        ZoneManager.updateRobotPositionAndVelocity(drive.getPose(), drive.getFieldRelativeVelocity());

        updateShootingState();
        updateOdometryEstimateWithLimelight();

        //Update the field telemetry's robot pose
        fieldTelemetry.setRobotPose(drive.getPose());

        DogLog.log("Current Zone", ZoneManager.getZone().name());
        DogLogUtil.logDouble("Turret Lock Error", shooter.turretLockError);

        //TODO: Drive telemetry for testing
        DogLog.log("Drive/ChassisSpeeds", drive.getRobotRelativeVelocity());
        DogLog.log("Drive/ModuleStates", drive.getModuleStates());
        DogLog.log("Drive/Rotation", drive.getPose().getRotation());
    }
}
