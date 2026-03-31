package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.DogLogUtil;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final CANcoder turretEncoder;

    private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    private final CANcoderConfiguration turretEncoderConfig = new CANcoderConfiguration();

    private final MotionMagicVoltage turretController = new MotionMagicVoltage(0);

    private final double kACCELERATION = 200.0;
    private final double kCRUISE_VELOCITY = 400.0;
    private final double kP = 100.0;
    private final double kI = 60.0;
    private final double kD = 0.0;

    //The unclamped value that the turret is commanded to go to (used to see if it is at the target)
    private double rawTargetDegrees = 0;

    private final double TURRET_ANGLE_TOLERANCE_DEGREES = 5.0;

    public final double MAX_TURRET_DEGREES = 121;
    private final double TURRET_HOME_DEGREES = 0;
    public final double MIN_TURRET_DEGREES = -54;

    private final double TURRET_ENCODER_DISCONTINUTY_POINT = 0.5;

    private final double TURRET_SENSOR_TO_MECHANISM_RATIO = 1;
    private final double TURRET_ROTOR_TO_SENSOR_RATIO = 45;

    private final double TURRET_ENCODER_MAGNET_OFFSET = 0.22900390625;

    private final int TURRET_MOTOR_ID = 7;
    private final int TURRET_ENCODER_ID = 8;

    private final int TURRET_STATOR_LIMIT = 35;
    
    private long lastMs = 0;

    public TurretSubsystem() {
        turretMotor = new TalonFX(TURRET_MOTOR_ID);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID);

        turretConfig.Feedback.RotorToSensorRatio = TURRET_ROTOR_TO_SENSOR_RATIO;
        turretConfig.Feedback.SensorToMechanismRatio = TURRET_SENSOR_TO_MECHANISM_RATIO;

        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretConfig.CurrentLimits.StatorCurrentLimit = TURRET_STATOR_LIMIT;

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
        rawTargetDegrees = degrees;
        turretMotor.setControl(turretController.withPosition(
            Units.degreesToRotations(MathUtil.clamp(degrees, MIN_TURRET_DEGREES, MAX_TURRET_DEGREES))
        ));
    }

    /*
     * Checks that the turret is not wrapping (error is relatively low) and that the target is reachable
     */
    public boolean isReady() {
        boolean targetInRange = 
            rawTargetDegrees < MAX_TURRET_DEGREES &&
            rawTargetDegrees > MIN_TURRET_DEGREES;

        return targetInRange &&
            Math.abs(getTurretAngleDegrees() - rawTargetDegrees) < TURRET_ANGLE_TOLERANCE_DEGREES;
    }

    public double getTurretAngleDegrees() {
        return Units.rotationsToDegrees(turretMotor.getPosition().getValueAsDouble());
    }

    public double getTargetAngleDegrees() {
        return Units.rotationsToDegrees(turretController.Position);
    }

    @Override
    public void periodic() {
        var turretAngle = getTurretAngleDegrees();
        
        DogLogUtil.logDoubleForceNT("Turret/turret_degrees", turretAngle);
        DogLogUtil.logDouble("Turret/turret_error_degrees", turretAngle - getTargetAngleDegrees());

        logMotorData();
    }

    private void logMotorData() {
        long currentMs = System.currentTimeMillis();
        if (currentMs - lastMs >= DogLogUtil.MOTOR_LOGGING_INTERVAL_MS) {
            BaseStatusSignal.refreshAll(
                turretMotor.getSupplyCurrent(), turretMotor.getStatorCurrent(), turretMotor.getDeviceTemp()
            );
            
            DogLogUtil.logDouble("Turret/turret_supply_current", turretMotor.getSupplyCurrent().getValueAsDouble());
            DogLogUtil.logDouble("Turret/turret_stator_current", turretMotor.getStatorCurrent().getValueAsDouble());
            DogLogUtil.logDouble("Turret/turret_device_temp", turretMotor.getDeviceTemp().getValueAsDouble());
            lastMs = currentMs;
        }
    }
}
