package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final TalonFX hoodMotor;

    private final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    private final CoastOut coast = new CoastOut();

    private final PositionVoltage hoodController = new PositionVoltage(0);
    private final MotionMagicVelocityVoltage shooterController = new MotionMagicVelocityVoltage(0);

    private final InterpolatingDoubleTreeMap shooterVelocityMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    //TODO: Find good tolerances
    private final double SHOOTER_VELOCITY_TOLERANCE_RPM = 5;
    private final double HOOD_TOLERANCE_ROTATIONS = 0.01;

    //TODO: Measure actual values
    private final double MAX_HOOD_ROTATIONS = 0;
    private final double MIN_HOOD_ROTATIONS = 0;

    //TODO: Tune all PID/Feedforward constants
    private final double SHOOTING_kP = 0.0;
    private final double SHOOTING_kI = 0.0;
    private final double SHOOTING_kD = 0.0;
    private final double SHOOTING_kV = 0.0;
    private final double SHOOTING_kS = 0.0;
    private final double SHOOTING_kACCELERATION = 0.0;

    private final double HOOD_kP = 0.0;
    private final double HOOD_kI = 0.0;
    private final double HOOD_kD = 0.0;

    //TODO: Find correct CAN IDs
    private final int SHOOTER_MOTOR_1_ID = 0;
    private final int SHOOTER_MOTOR_2_ID = 0;
    private final int HOOD_MOTOR_ID = 0;

    //Supplier that allows us to encapsulate shoot-on-the-move behavior outside of this class
    private final DoubleSupplier distanceFromHubSupplier;

    public ShooterSubsystem(DoubleSupplier distanceFromHubSupplier) {
        this.distanceFromHubSupplier = distanceFromHubSupplier;

        shooterMotor1 = new TalonFX(SHOOTER_MOTOR_1_ID);
        shooterMotor2 = new TalonFX(SHOOTER_MOTOR_2_ID);

        hoodMotor = new TalonFX(HOOD_MOTOR_ID);

        shooterConfig.Slot0.kP = SHOOTING_kP;
        shooterConfig.Slot0.kI = SHOOTING_kI;
        shooterConfig.Slot0.kD = SHOOTING_kD;
        shooterConfig.Slot0.kV = SHOOTING_kV;
        shooterConfig.Slot0.kS = SHOOTING_kS;

        shooterConfig.MotionMagic.MotionMagicAcceleration = SHOOTING_kACCELERATION;

        hoodConfig.Slot0.kP = HOOD_kP;
        hoodConfig.Slot0.kI = HOOD_kI;
        hoodConfig.Slot0.kD = HOOD_kD;

        shooterMotor1.getConfigurator().apply(shooterConfig);
        shooterMotor2.getConfigurator().apply(shooterConfig);

        hoodMotor.getConfigurator().apply(hoodConfig);

        shooterMotor2.setControl(new Follower(SHOOTER_MOTOR_1_ID, MotorAlignmentValue.Opposed));

        buildInterpolationMaps();
    }

    private void buildInterpolationMaps() {
        shooterVelocityMap.put(0.0, 0.0);

        hoodAngleMap.put(0.0, 0.0);
    }

    public Command resetSubsystem() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> hoodMotor.setPosition(0))
        );
    }

    public Command homeSubsystem() {
        return new SequentialCommandGroup(
            stop(),
            new InstantCommand(() -> hoodMotor.setControl(hoodController.withPosition(0)))
        );
    }

    /**
     * Spin up the shooter based on our distance from the goal
     */
    public Command spinUp() {
        return spinToRPM(() -> shooterVelocityMap.get(distanceFromHubSupplier.getAsDouble()));
    }

    /**
     * Set the flywheel to coast to preserve momentum while not drawing any current
     */
    public Command stop() {
        return runOnce(() -> shooterMotor1.setControl(coast));
    }

    /**
     * Default command to constantly aim the hood to shoot
     */
    public Command hoodDefaultCommand() {
        return run(
            () -> hoodMotor.setControl(
                hoodController.withPosition(MathUtil.clamp(hoodAngleMap.get(distanceFromHubSupplier.getAsDouble()), MIN_HOOD_ROTATIONS, MAX_HOOD_ROTATIONS))
            )
        );
    }

    public Command setHoodPosition(DoubleSupplier rotations) {
        return runOnce(
            () -> hoodMotor.setControl(hoodController.withPosition(MathUtil.clamp(rotations.getAsDouble(), MIN_HOOD_ROTATIONS, MAX_HOOD_ROTATIONS)))
        ).andThen(new WaitUntilCommand(() -> hoodAtPosition()));
    }

    private Command spinToRPM(DoubleSupplier rpm) {
        return runOnce(
            () -> shooterMotor1.setControl(shooterController.withVelocity(rpm.getAsDouble() / 60))
        ).andThen(new WaitUntilCommand(() -> atVelocity()));
    }

    private boolean hoodAtPosition() {
        return Math.abs(hoodMotor.getPosition().getValueAsDouble() - hoodController.Position) < HOOD_TOLERANCE_ROTATIONS;
    }

    private boolean atVelocity() {
        return Math.abs(getRPM() - getTargetRPM()) < SHOOTER_VELOCITY_TOLERANCE_RPM;
    }

    /**
     *  @return the shooter's velocity in Rotations Per Second
     */ 
    private double getRPM() {
        return shooterMotor1.getVelocity().getValueAsDouble() / 60.0;
    }

    /**
     *  @return the shooter's target velocity in Rotations Per Second
     */ 
    private double getTargetRPM() {
        return shooterController.Velocity / 60.0;
    }

    @Override
    public void periodic() {
        DogLog.forceNt.log("Shooter/shooter_velocity", getRPM());
        DogLog.forceNt.log("Shooter/shooter_target_velocity", getTargetRPM());
        DogLog.forceNt.log("Shooter/shooter_at_velocity", atVelocity());

        DogLog.forceNt.log("Shooter/hood_at_position", hoodAtPosition());
    }
}
