package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX rollerMotor;
    private final TalonFX deployMotor;

    private final CANcoder deployEncoder;
    private final CANcoderConfiguration deployEncoderConfig = new CANcoderConfiguration();

    private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration deployConfig = new TalonFXConfiguration();

    private final PositionVoltage deployController = new PositionVoltage(0);

    private final double DEPLOY_ENCODER_ZERO_OFFSET = -0.4013671875;

    private final double DEPLOY_ENCODER_TO_MECHANISM_RATIO = 1.0;
    private final double DEPLOY_MOTOR_TO_ENCODER_RATIO = 52.0;

    private final double TURRET_ENCODER_DISCONTINUTY_POINT = 0.651;

    private final double DEPLOYED_ROTATIONS = 0;
    private final double RETRACTED_ROTATIONS = 0.31;

    private final double DEPLOY_kP = 25.0;
    private final double DEPLOY_kI = 0.0;
    private final double DEPLOY_kD = 0.0;
    private final double DEPLOY_kV = 100;

    private final double RETRACT_kP = 35.0;
    private final double RETRACT_kI = 0.1;
    private final double RETRACT_kD = 0;
    private final double RETRACT_kV = 100;

    private final int ROLLER_MOTOR_ID = 16;
    private final int DEPLOY_MOTOR_ID = 17;
    private final int DEPLOY_ENCODER_ID = 18;

    public IntakeSubsystem() {
        rollerMotor = new TalonFX(ROLLER_MOTOR_ID);
        deployMotor = new TalonFX(DEPLOY_MOTOR_ID);
        deployEncoder = new CANcoder(DEPLOY_ENCODER_ID);

        //Roller motor config
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rollerMotor.getConfigurator().apply(rollerConfig);

        //CANCoder config
        deployEncoderConfig.MagnetSensor.MagnetOffset = DEPLOY_ENCODER_ZERO_OFFSET;
        deployEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        deployEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = TURRET_ENCODER_DISCONTINUTY_POINT;

        deployEncoder.getConfigurator().apply(deployEncoderConfig);
        
        //Deploy motor config
        deployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        deployConfig.Feedback.FeedbackRemoteSensorID = deployEncoder.getDeviceID();
        deployConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        deployConfig.Feedback.SensorToMechanismRatio = DEPLOY_ENCODER_TO_MECHANISM_RATIO;
        deployConfig.Feedback.RotorToSensorRatio = DEPLOY_MOTOR_TO_ENCODER_RATIO;

        deployConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        deployConfig.CurrentLimits.StatorCurrentLimit = 100;

        deployConfig.Slot0.kP = RETRACT_kP;
        deployConfig.Slot0.kI = RETRACT_kI;
        deployConfig.Slot0.kD = RETRACT_kD;
        deployConfig.Slot0.kV = RETRACT_kV;
        
        deployConfig.Slot1.kP = DEPLOY_kP;
        deployConfig.Slot1.kI = DEPLOY_kI;
        deployConfig.Slot1.kD = DEPLOY_kD;
        deployConfig.Slot1.kV = DEPLOY_kV;

        deployMotor.getConfigurator().apply(deployConfig);

        //Explictly set the deploy motor position
        deployMotor.setPosition(deployEncoder.getAbsolutePosition().getValue().times(DEPLOY_MOTOR_TO_ENCODER_RATIO));

        retract();
    }

    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public void stop() {
        rollerMotor.set(0);
    }

    public void deploy() {
        deployMotor.setControl(deployController.withSlot(1).withPosition(DEPLOYED_ROTATIONS));
    }

    public void retract() {
        deployMotor.setControl(deployController.withSlot(0).withPosition(RETRACTED_ROTATIONS));
    }

    @Override
    public void periodic() {
        DogLog.log("Intake/roller_speed", rollerMotor.get());
        DogLog.log("Intake/intake_rotations", deployMotor.getPosition().getValueAsDouble());
    }
}
