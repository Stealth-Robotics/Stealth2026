package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggingUtility;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final CANcoder turretEncoder;

    private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    private final CANcoderConfiguration turretEncoderConfig = new CANcoderConfiguration();

    private final MotionMagicVoltage turretController = new MotionMagicVoltage(0);

    private final StatusSignal<Angle> turretPosition;
    private final StatusSignal<AngularVelocity> turretVelocitySignal;
    private final StatusSignal<Current> turretSupplyCurrent;
    private final StatusSignal<Current> turretStatorCurrent;
    private final StatusSignal<Temperature> turretDeviceTemp;

    private final double TURRET_LOOKAHEAD_SECONDS = 0.1;

    private final double kACCELERATION = 200.0;
    private final double kCRUISE_VELOCITY = 400.0;
    private final double kP = 120.0;
    private final double kI = 80.0;
    private final double kD = 0.0;
    private final double kV = 5.4; //Theoretical value

    //The unclamped value that the turret is commanded to go to (used to see if it is at the target)
    private double rawTargetDegrees = 0;

    private final double TURRET_ANGLE_TOLERANCE_DEGREES = 8.0;

    public final double MAX_TURRET_DEGREES = 120;
    private final double TURRET_HOME_DEGREES = 0;
    public final double MIN_TURRET_DEGREES = -53;

    private final double TURRET_ENCODER_DISCONTINUTY_POINT = 0.5;

    private final double TURRET_SENSOR_TO_MECHANISM_RATIO = 1;
    private final double TURRET_ROTOR_TO_SENSOR_RATIO = 45;

    private final double TURRET_ENCODER_MAGNET_OFFSET = 0.22900390625;

    private final int TURRET_MOTOR_ID = 7;
    private final int TURRET_ENCODER_ID = 8;

    private final int TURRET_STATOR_LIMIT = 35;
    private final int TURRET_SUPPLY_LIMIT = 30;
    
    public TurretSubsystem() {
        turretMotor = new TalonFX(TURRET_MOTOR_ID);
        turretEncoder = new CANcoder(TURRET_ENCODER_ID);

        turretConfig.Feedback.RotorToSensorRatio = TURRET_ROTOR_TO_SENSOR_RATIO;
        turretConfig.Feedback.SensorToMechanismRatio = TURRET_SENSOR_TO_MECHANISM_RATIO;

        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretConfig.CurrentLimits.StatorCurrentLimit = TURRET_STATOR_LIMIT;
        turretConfig.CurrentLimits.SupplyCurrentLimit = TURRET_SUPPLY_LIMIT;

        turretConfig.Slot0.kP = kP;
        turretConfig.Slot0.kI = kI;
        turretConfig.Slot0.kD = kD;
        turretConfig.Slot0.kV = kV;
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

        //Software limits to be sure the turret doesn't go past its physical range
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(MAX_TURRET_DEGREES);

        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(MIN_TURRET_DEGREES);

        turretMotor.getConfigurator().apply(turretConfig);
        turretEncoder.getConfigurator().apply(turretEncoderConfig);
        
        turretPosition = turretMotor.getPosition();
        turretVelocitySignal = turretMotor.getVelocity();
        turretSupplyCurrent = turretMotor.getSupplyCurrent();
        turretStatorCurrent = turretMotor.getStatorCurrent();
        turretDeviceTemp = turretMotor.getDeviceTemp();
        
        //High priority
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            turretPosition, turretVelocitySignal
        );

        //Low priority
        BaseStatusSignal.setUpdateFrequencyForAll(
            10.0,
            turretSupplyCurrent, turretStatorCurrent, turretDeviceTemp
        );
    }

    public void homeTurret() {
        setTarget(TURRET_HOME_DEGREES, 0.0);
    }

    public void setTarget(double degrees, double velocity) {
        rawTargetDegrees = degrees;

        double clampedDegrees = MathUtil.clamp(degrees, MIN_TURRET_DEGREES, MAX_TURRET_DEGREES);
        double targetRotations = Units.degreesToRotations(clampedDegrees);

        double feedforward = Units.degreesToRotations(velocity) * kV;

        //Protect against the feedforward voltage trying to rotate the turret past its limits
        double currentAngle = getTurretAngleDegrees();
        if ((currentAngle >= MAX_TURRET_DEGREES && feedforward > 0) ||
            (currentAngle <= MIN_TURRET_DEGREES && feedforward < 0))
            feedforward = 0.0;

        turretMotor.setControl(
            turretController
                .withPosition(targetRotations)
                .withFeedForward(feedforward)
        );
    }

    /*
     * Checks that the turret is not wrapping (error is relatively low) and that the target is reachable
     */
    public boolean isReady() {
        boolean targetInRange = 
            rawTargetDegrees < (MAX_TURRET_DEGREES + TURRET_ANGLE_TOLERANCE_DEGREES) &&
            rawTargetDegrees > (MIN_TURRET_DEGREES - TURRET_ANGLE_TOLERANCE_DEGREES);

        return targetInRange && !isApproachingLimit() &&
            Math.abs(getTurretAngleDegrees() - rawTargetDegrees) < TURRET_ANGLE_TOLERANCE_DEGREES;
    }

    public boolean isApproachingLimit() {
        double futureAngle = getTurretAngleDegrees() + (getTurretVelocity() * TURRET_LOOKAHEAD_SECONDS);
        return futureAngle <= MIN_TURRET_DEGREES || futureAngle >= MAX_TURRET_DEGREES;
    }

    private double getTurretVelocity() {
        return Units.rotationsToDegrees(turretVelocitySignal.getValueAsDouble());
    }

    public double getTurretAngleDegrees() {
        return Units.rotationsToDegrees(turretPosition.getValueAsDouble());
    }

    public double getTargetAngleDegrees() {
        return Units.rotationsToDegrees(turretController.Position);
    }

    @Override
    public void periodic() {
        //Bulk refresh the high priority signals
        BaseStatusSignal.refreshAll(turretPosition, turretVelocitySignal);

        var turretAngle = getTurretAngleDegrees();
        
        LoggingUtility.logDoubleForceNT("Turret/turret_degrees", turretAngle);
        LoggingUtility.logDouble("Turret/turret_error_degrees", turretAngle - getTargetAngleDegrees());

        if (LoggingUtility.LOG_TURRET && LoggingUtility.updateLowPriorityLogs()) {
            BaseStatusSignal.refreshAll(
                turretSupplyCurrent, turretStatorCurrent, turretDeviceTemp
            );
            
            LoggingUtility.logDouble("Turret/turret_supply_current", turretSupplyCurrent.getValueAsDouble());
            LoggingUtility.logDouble("Turret/turret_stator_current", turretStatorCurrent.getValueAsDouble());
            LoggingUtility.logDouble("Turret/turret_device_temp", turretDeviceTemp.getValueAsDouble());
        }
    }
}
