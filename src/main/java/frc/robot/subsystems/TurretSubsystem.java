package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final CANcoder turretEncoder;

    private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    private final CANcoderConfiguration turretEncoderConfig = new CANcoderConfiguration();

    private final MotionMagicVoltage turretController = new MotionMagicVoltage(0);

    //TODO: Tune PID/Feedforward constants
    private final double kACCELERATION = 0.0;
    private final double kCRUISE_VELOCITY = 0.0;
    private final double kP = 0.0;
    private final double kI = 0.0;
    private final double kD = 0.0;

    //TODO: Find a good tolerance
    private final double TURRET_ANGLE_TOLERANCE_DEGREES = 0.25;

    //TODO: Find actual values
    private final double MAX_TURRET_DEGREES = 0;
    private final double MIN_TURRET_DEGREES = 0;

    //Figure out mechanism ratio
    private final double ENCODER_TO_TURRET_RATIO = 0;
    private final double MOTOR_TO_ENCODER_RATIO = 1;

    //TODO: Find zeroed value
    private final double TURRET_ENCODER_MAGNET_OFFSET = 0;

    //TODO: Find correct CAN IDs
    private final int TURRET_MOTOR_ID = 0;
    private final int TURRET_ENCODER_ID = 0;

    public TurretSubsystem() {
        turretMotor = new TalonFX(TURRET_MOTOR_ID);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID);

        turretConfig.Feedback.RotorToSensorRatio = MOTOR_TO_ENCODER_RATIO;
        turretConfig.Feedback.SensorToMechanismRatio = ENCODER_TO_TURRET_RATIO;

        turretConfig.Slot0.kP = kP;
        turretConfig.Slot0.kI = kI;
        turretConfig.Slot0.kD = kD;
        turretConfig.MotionMagic.MotionMagicAcceleration = kACCELERATION;
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = kCRUISE_VELOCITY;

        turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        //Cancoder configuration
        turretConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
        turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        turretEncoderConfig.MagnetSensor.MagnetOffset = TURRET_ENCODER_MAGNET_OFFSET;
        turretEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        turretMotor.getConfigurator().apply(turretConfig);
        turretEncoder.getConfigurator().apply(turretEncoderConfig);
    }

    public Command rotateToAngle(DoubleSupplier degrees) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> turretMotor.setControl(turretController.withPosition(
                    MathUtil.clamp(degrees.getAsDouble(), MIN_TURRET_DEGREES, MAX_TURRET_DEGREES)
                ))
            ),
            new WaitUntilCommand(this::isTurretAtAngle)
        );
    }

    public boolean isTurretAtAngle() {
        return Math.abs(getTurretAngleDegrees() - getTargetAngleDegrees()) < TURRET_ANGLE_TOLERANCE_DEGREES;
    }

    private double getTurretAngleDegrees() {
        return Units.rotationsToDegrees(turretMotor.getPosition().getValueAsDouble());
    }

    private double getTargetAngleDegrees() {
        return Units.rotationsToDegrees(turretController.Position);
    }

    @Override
    public void periodic() {
        DogLog.forceNt.log("Turret/turret_degrees", getTurretAngleDegrees());
        DogLog.forceNt.log("Turret/turret_target_degrees", getTargetAngleDegrees());
    }
}
