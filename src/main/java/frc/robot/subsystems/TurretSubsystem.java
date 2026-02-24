package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final CANcoder turretEncoder;

    private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    private final CANcoderConfiguration turretEncoderConfig = new CANcoderConfiguration();

    private final MotionMagicVoltage turretController = new MotionMagicVoltage(0);

    private final double kACCELERATION = 80.0;
    private final double kCRUISE_VELOCITY = 300.0;
    private final double kP = 80.0;
    private final double kI = 1.0;
    private final double kD = 0.0;

    private final double TURRET_ANGLE_TOLERANCE_DEGREES = 5;

    private final double MAX_TURRET_DEGREES = 121;
    private final double TURRET_HOME_DEGREES = 0;
    private final double MIN_TURRET_DEGREES = -36;

    private final double TURRET_ENCODER_DISCONTINUTY_POINT = 0.5;

    private final double TURRET_SENSOR_TO_MECHANISM_RATIO = 1;
    private final double TURRET_ROTOR_TO_SENSOR_RATIO = 45;

    private final double TURRET_ENCODER_MAGNET_OFFSET = 0.439697;

    private final int TURRET_MOTOR_ID = 7;
    private final int TURRET_ENCODER_ID = 8;

    public TurretSubsystem() {
        turretMotor = new TalonFX(TURRET_MOTOR_ID);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID);

        turretConfig.Feedback.RotorToSensorRatio = TURRET_ROTOR_TO_SENSOR_RATIO;
        turretConfig.Feedback.SensorToMechanismRatio = TURRET_SENSOR_TO_MECHANISM_RATIO;

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
        turretEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = TURRET_ENCODER_DISCONTINUTY_POINT;

        turretMotor.getConfigurator().apply(turretConfig);
        turretEncoder.getConfigurator().apply(turretEncoderConfig);
    }

    public void homeTurret() {
        setTargetDegrees(TURRET_HOME_DEGREES);
    }

    public void setTargetDegrees(double degrees) {
        turretMotor.setControl(turretController.withPosition(
            Units.degreesToRotations(MathUtil.clamp(degrees, MIN_TURRET_DEGREES, MAX_TURRET_DEGREES))
        ));
    }

    public boolean isTurretNearAngle() {
        return Math.abs(getTurretAngleDegrees() - getTargetAngleDegrees()) < TURRET_ANGLE_TOLERANCE_DEGREES;
    }

    public double getTurretAngleDegrees() {
        return Units.rotationsToDegrees(turretMotor.getPosition().getValueAsDouble());
    }

    public double getTargetAngleDegrees() {
        return Units.rotationsToDegrees(turretController.Position);
    }

    @Override
    public void periodic() {
        DogLog.log("Turret/turret_degrees", Math.round(getTurretAngleDegrees() * 100) / 100.0);
        DogLog.log("Turret/turret_target_degrees", Math.round(getTargetAngleDegrees() * 100) / 100.0);
    }
}
