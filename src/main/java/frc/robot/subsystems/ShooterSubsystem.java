package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    private final VelocityVoltage shooterController = new VelocityVoltage(0);

    private final double HOOD_ENCODER_MAGNET_OFFSET = -0.0248;
    private final double HOOD_ENCODER_DISCONTINUTY_POINT = 1;

    private final double HOOD_ROTOR_TO_SENSOR_RATIO = 5.0;
    private final double HOOD_SENSOR_TO_MECHANISM_RATIO = 8.0;

    //TODO: Find good tolerance
    private final double SHOOTER_VELOCITY_TOLERANCE_RPM = 25;

    private final double MAX_HOOD_DEGREES = 21;
    private final double MIN_HOOD_DEGREES = 0;

    private final double SHOOTER_MOTOR_TO_FLYWHEEL_RATIO = 1.5;

    private final double SHOOTING_kP = 0.076017;
    private final double SHOOTING_kI = 0.0;
    private final double SHOOTING_kD = 0.0;
    private final double SHOOTING_kA = 0.0072909;
    private final double SHOOTING_kV = 0.18437;
    private final double SHOOTING_kS = 0.25233;

    private final double HOOD_kP = 150.0;
    private final double HOOD_kI = 10.0;
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
        shooterConfig.Feedback.SensorToMechanismRatio = SHOOTER_MOTOR_TO_FLYWHEEL_RATIO;

        shooterConfig.Slot0.kP = SHOOTING_kP;
        shooterConfig.Slot0.kI = SHOOTING_kI;
        shooterConfig.Slot0.kD = SHOOTING_kD;
        shooterConfig.Slot0.kV = SHOOTING_kV;
        shooterConfig.Slot0.kS = SHOOTING_kS;
        shooterConfig.Slot0.kA = SHOOTING_kA;

        //Hood motor configuration
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        hoodConfig.Feedback.RotorToSensorRatio = HOOD_ROTOR_TO_SENSOR_RATIO;
        hoodConfig.Feedback.SensorToMechanismRatio = HOOD_SENSOR_TO_MECHANISM_RATIO;

        hoodConfig.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

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
     * Set the flywheel to coast to preserve rotational inertia
     */
    public void coastShooter() {
        shooterMotor1.setControl(coast);
    }

    /**
     * Sets the hood to the specified angle in degrees
     */
    public void setHoodDegrees(double degrees) {
        hoodMotor.setControl(hoodController.withPosition(
            Units.degreesToRotations(MathUtil.clamp(degrees, MIN_HOOD_DEGREES, MAX_HOOD_DEGREES)))
        );
    }

    /**
     * Sets the shooter to spin up to the specified RPM
     */
    public void spinToRPM(double rpm) {
        shooterMotor1.setControl(shooterController.withVelocity(rpm / 60.0));
    }

    /**
     * @return Whether or not the shooter is at its target velocity (within a tolerance)
     */
    public boolean isShooterAtVelocity() {
        return Math.abs(getRPM() - getTargetRPM()) < SHOOTER_VELOCITY_TOLERANCE_RPM;
    }

    /**
     *  @return the shooter's velocity in RPM
     */ 
    private double getRPM() {
        return shooterMotor1.getVelocity().getValueAsDouble() * 60.0;
    }

    /**
     *  @return the shooter's target velocity in RPM
     */ 
    private double getTargetRPM() {
        return shooterController.Velocity * 60.0;
    }

    public double getHoodDegrees() {
        return Units.rotationsToDegrees(hoodMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        DogLog.log("Shooter/shooter_rpm", getRPM());
        DogLog.log("Shooter/shooter_target_rpm", getTargetRPM());
        
        DogLog.log("Shooter/hood_encoder", hoodEncoder.getAbsolutePosition().getValueAsDouble());
        DogLog.log("Shooter/hood_angle", Math.round(Units.rotationsToDegrees(hoodMotor.getPosition().getValueAsDouble()) * 100) / 100.0);
    }
}