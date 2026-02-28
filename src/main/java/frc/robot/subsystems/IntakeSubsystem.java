package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX rollerMotor;
    private final TalonFX deployMotor;

    private final CANcoder deployEncoder;
    private final CANcoderConfiguration deployEncoderConfig = new CANcoderConfiguration();

    private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration deployConfig = new TalonFXConfiguration();

    private final MotionMagicVoltage deployController = new MotionMagicVoltage(0);

    //TODO: Rezero encoder so that zero is when the intake is horizontal
    private final double DEPLOY_ENCODER_ZERO_OFFSET = -0.305664;

    private final double DEPLOY_ENCODER_TO_MECHANISM_RATIO = 1.0;
    private final double DEPLOY_MOTOR_TO_ENCODER_RATIO = 52.0;

    private final double TURRET_ENCODER_DISCONTINUTY_POINT = 1;

    private final double DEPLOYED_ROTATIONS = 0;
    private final double RETRACTED_ROTATIONS = 0.29;

    //TODO: Tune MotionMagic & PID constants
    private final double DEPLOY_kP = 0.0;
    private final double DEPLOY_kI = 0.0;
    private final double DEPLOY_kD = 0.0;
    private final double DEPLOY_KG = 0.0;
    private final double DEPLOY_MOTIONMAGIC_kACCELERATION = 2.0;
    private final double DEPLOY_MOTIONMAGIC_kVELOCITY = 0.5;

    private final int ROLLER_MOTOR_ID = 16;
    private final int DEPLOY_MOTOR_ID = 17;
    private final int DEPLOY_ENCODER_ID = 18;

    public IntakeSubsystem() {
        rollerMotor = new TalonFX(ROLLER_MOTOR_ID);
        deployMotor = new TalonFX(DEPLOY_MOTOR_ID);
        deployEncoder = new CANcoder(DEPLOY_ENCODER_ID);

        //CANCoder config
        deployEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = TURRET_ENCODER_DISCONTINUTY_POINT;
        deployEncoderConfig.MagnetSensor.MagnetOffset = DEPLOY_ENCODER_ZERO_OFFSET;
        deployEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        //Roller motor config
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        //Deploy motor config
        deployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        deployConfig.Feedback.FeedbackRemoteSensorID = deployEncoder.getDeviceID();
        deployConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        deployConfig.Feedback.SensorToMechanismRatio = DEPLOY_ENCODER_TO_MECHANISM_RATIO;
        deployConfig.Feedback.RotorToSensorRatio = DEPLOY_MOTOR_TO_ENCODER_RATIO;

        deployConfig.Slot0.kP = DEPLOY_kP;
        deployConfig.Slot0.kI = DEPLOY_kI;
        deployConfig.Slot0.kD = DEPLOY_kD;
        deployConfig.Slot0.kG = DEPLOY_KG;

        deployConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        deployConfig.MotionMagic.MotionMagicAcceleration = DEPLOY_MOTIONMAGIC_kACCELERATION;
        deployConfig.MotionMagic.MotionMagicCruiseVelocity = DEPLOY_MOTIONMAGIC_kVELOCITY;

        rollerMotor.getConfigurator().apply(rollerConfig);
        deployMotor.getConfigurator().apply(deployConfig);
        deployEncoder.getConfigurator().apply(deployEncoderConfig);

        //Explictly set the intake's deploy motor position on startup
        deployMotor.setPosition(deployEncoder.getAbsolutePosition().getValue());

        retract();
    }

    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public void stop() {
        rollerMotor.set(0);
    }

    public void deploy() {
        deployMotor.setControl(deployController.withPosition(DEPLOYED_ROTATIONS));
    }

    public void retract() {
        deployMotor.setControl(deployController.withPosition(RETRACTED_ROTATIONS));
    }

    private double getIntakeRotations() {
        return deployMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        DogLog.log("Intake/roller_speed", rollerMotor.get());
        DogLog.log("Intake/target_position", getIntakeRotations());

        DogLog.log("Intake/intake_rotations", getIntakeRotations());
    }
}
