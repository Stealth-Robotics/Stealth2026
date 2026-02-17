package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;

    private final TalonFX hoodMotor;
    private final CANcoder hoodEncoder;

    private final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    private final CANcoderConfiguration hoodEncoderConfig = new CANcoderConfiguration();

    private final CoastOut coast = new CoastOut();

    private final PositionVoltage hoodController = new PositionVoltage(0);
    private final MotionMagicVelocityVoltage shooterController = new MotionMagicVelocityVoltage(0);

    private final double HOOD_ENCODER_MAGNET_OFFSET = -0.054688;
    private final double HOOD_ENCODER_DISCONTINUTY_POINT = 1;

    private final double HOOD_ROTOR_TO_SENSOR_RATIO = 1.0 / 5.0;
    private final double HOOD_SENSOR_TO_MECHANISM_RATIO = 1.0 / 8.0;

    //TODO: Find good tolerances
    private final double SHOOTER_VELOCITY_TOLERANCE_RPM = 5;
    private final double HOOD_TOLERANCE_ROTATIONS = 0.01;

    private final double MAX_HOOD_ROTATIONS = 0.47;
    private final double MIN_HOOD_ROTATIONS = 0;

    //TODO: Tune all PID/Feedforward constants
    private final double SHOOTING_kP = 0.0;
    private final double SHOOTING_kI = 0.0;
    private final double SHOOTING_kD = 0.0;
    private final double SHOOTING_kA = 0.0;
    private final double SHOOTING_kV = 0.0;
    private final double SHOOTING_kS = 0.0;
    private final double SHOOTING_MOTIONMAGIC_kACCELERATION = 0.0;

    //TODO: Tune kP and kI until satisfactory
    private final double HOOD_kP = 10.0;
    private final double HOOD_kI = 0.0;
    private final double HOOD_kD = 0.0;

    private final int SHOOTER_MOTOR_1_ID = 2;
    private final int SHOOTER_MOTOR_2_ID = 3;

    private final int HOOD_MOTOR_ID = 4;
    private final int HOOD_ENCODER_ID = 9;

    public ShooterSubsystem() {
        shooterMotor1 = new TalonFX(SHOOTER_MOTOR_1_ID);
        shooterMotor2 = new TalonFX(SHOOTER_MOTOR_2_ID);

        hoodMotor = new TalonFX(HOOD_MOTOR_ID);
        hoodEncoder = new CANcoder(HOOD_ENCODER_ID);

        //Shooter motors configuration
        shooterConfig.Slot0.kP = SHOOTING_kP;
        shooterConfig.Slot0.kI = SHOOTING_kI;
        shooterConfig.Slot0.kD = SHOOTING_kD;
        shooterConfig.Slot0.kV = SHOOTING_kV;
        shooterConfig.Slot0.kS = SHOOTING_kS;
        shooterConfig.Slot0.kA = SHOOTING_kA;

        shooterConfig.MotionMagic.MotionMagicAcceleration = SHOOTING_MOTIONMAGIC_kACCELERATION;

        //Hood motor configuration
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        hoodConfig.Feedback.RotorToSensorRatio = HOOD_ROTOR_TO_SENSOR_RATIO;
        hoodConfig.Feedback.SensorToMechanismRatio = HOOD_SENSOR_TO_MECHANISM_RATIO;

        hoodConfig.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        hoodConfig.Slot0.kP = HOOD_kP;
        hoodConfig.Slot0.kI = HOOD_kI;
        hoodConfig.Slot0.kD = HOOD_kD;
        
        //Cancoder configuration
        hoodEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = HOOD_ENCODER_DISCONTINUTY_POINT;
        hoodEncoderConfig.MagnetSensor.MagnetOffset = HOOD_ENCODER_MAGNET_OFFSET;
        hoodEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        //Apply device configs
        shooterMotor1.getConfigurator().apply(shooterConfig);
        shooterMotor2.getConfigurator().apply(shooterConfig);
        
        hoodMotor.getConfigurator().apply(hoodConfig);
        hoodEncoder.getConfigurator().apply(hoodEncoderConfig);

        //Set the other shooting motor to follow the other (but inverted)
        shooterMotor2.setControl(new Follower(SHOOTER_MOTOR_1_ID, MotorAlignmentValue.Opposed));
    }

    /**
     * Commands the subsystem to its homed behavior (called on teleop init and autonomous init)
     */
    public Command homeSubsystem() {
        return new SequentialCommandGroup(
            deactivateShooter(),
            setHoodPosition(() -> MIN_HOOD_ROTATIONS)
        );
    }

    /**
     * Set the flywheel to coast to preserve rotational inertia
     */
    public Command deactivateShooter() {
        return runOnce(() -> shooterMotor1.setControl(coast));
    }

    /**
     * Sets the hood to the specified rotations and waits until finished
     */
    public Command setHoodPosition(DoubleSupplier rotations) {
        return runOnce(
            () -> hoodMotor.setControl(hoodController.withPosition(
                MathUtil.clamp(rotations.getAsDouble(), MIN_HOOD_ROTATIONS, MAX_HOOD_ROTATIONS))
            )
        ).andThen(new WaitUntilCommand(() -> isHoodAtPosition()));
    }

    /**
     * Sets the shooter to spin up to the specified RPM and waits until at that speed
     */
    public Command spinToRPM(DoubleSupplier rpm) {
        return runOnce(
            () -> shooterMotor1.setControl(shooterController.withVelocity(rpm.getAsDouble() / 60))
        ).andThen(new WaitUntilCommand(() -> isShooterAtVelocity()));
    }
    
    /**
     * @return Whether or not the hood is at its target rotations (within a tolerance)
     */
    private boolean isHoodAtPosition() {
        return Math.abs(hoodMotor.getPosition().getValueAsDouble() - hoodController.Position) < HOOD_TOLERANCE_ROTATIONS;
    }

    /**
     * @return Whether or not the shooter is at its target velocity (within a tolerance)
     */
    public boolean isShooterAtVelocity() {
        return Math.abs(getRPM() - getTargetRPM()) < SHOOTER_VELOCITY_TOLERANCE_RPM;
    }

    /**
     *  @return the shooter's velocity in Rotations Per Minute
     */ 
    private double getRPM() {
        return shooterMotor1.getVelocity().getValueAsDouble() / 60.0;
    }

    /**
     *  @return the shooter's target velocity in Rotations Per Minute
     */ 
    private double getTargetRPM() {
        return shooterController.Velocity / 60.0;
    }

    @Override
    public void periodic() {
        DogLog.forceNt.log("Shooter/shooter_rpm", getRPM());
        DogLog.forceNt.log("Shooter/shooter_target_rpm", getTargetRPM());
        DogLog.forceNt.log("Shooter/shooter_ready", isShooterAtVelocity());

        DogLog.forceNt.log("Shooter/hood_ready", isHoodAtPosition());
    }
}