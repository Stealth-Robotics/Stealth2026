package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.DogLogUtil;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX rollerMotor;
    private final TalonFX deployMotor;

    private final CANcoder deployEncoder;
    private final CANcoderConfiguration deployEncoderConfig = new CANcoderConfiguration();

    private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration deployConfig = new TalonFXConfiguration();

    private final MotionMagicVoltage deployController = new MotionMagicVoltage(0);
    private final VoltageOut rollerController = new VoltageOut(0);

    private final double INTAKE_ROLLER_VOLTAGE = 7;
    private final double MAX_ROLLER_SPEED = 0.8;

    private final double DEPLOY_ENCODER_ZERO_OFFSET = -0.4013671875;

    private final double DEPLOY_ENCODER_TO_MECHANISM_RATIO = 1.0;
    private final double DEPLOY_MOTOR_TO_ENCODER_RATIO = 52.0;

    private final double TURRET_ENCODER_DISCONTINUTY_POINT = 0.651;

    private final double DEPLOYED_ROTATIONS = 0;
    private final double RETRACTED_ROTATIONS = 0.308;

    private final double DEPLOY_kP = 30;
    private final double RETRACT_kP = 35;
    private final double DEPLOY_kI = 0;
    private final double RETRACT_kI = 0.09;
    private final double DEPLOY_kACCEL = 10;
    private final double DEPLOY_kVELO = 50;

    private final int ROLLER_MOTOR_ID = 16;
    private final int DEPLOY_MOTOR_ID = 17;
    private final int DEPLOY_ENCODER_ID = 18;

    private final int DEPLOY_STATOR_LIMIT = 20;
    private final int ROLLER_STATOR_LIMIT = 100;

    private final double INTAKE_TOSS_INTERVAL_SECONDS = 0.5;
    private final double INTAKE_TOSS_PERCENTAGE = 1.0;

    public IntakeSubsystem() {
        rollerMotor = new TalonFX(ROLLER_MOTOR_ID);
        deployMotor = new TalonFX(DEPLOY_MOTOR_ID);
        deployEncoder = new CANcoder(DEPLOY_ENCODER_ID);

        //Roller motor config
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rollerConfig.CurrentLimits.StatorCurrentLimit = ROLLER_STATOR_LIMIT;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

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
        deployConfig.CurrentLimits.StatorCurrentLimit = DEPLOY_STATOR_LIMIT;

        deployConfig.Slot0.kP = RETRACT_kP;
        deployConfig.Slot0.kI = RETRACT_kI;

        deployConfig.Slot1.kP = DEPLOY_kP;
        deployConfig.Slot1.kI = DEPLOY_kI;
        
        deployConfig.MotionMagic.MotionMagicAcceleration = DEPLOY_kACCEL;
        deployConfig.MotionMagic.MotionMagicCruiseVelocity = DEPLOY_kVELO;

        deployMotor.getConfigurator().apply(deployConfig);

        deployMotor.setControl(deployController.withSlot(0).withPosition(deployMotor.getPosition().getValue()));
        DogLog.log("Intake/roller_max_current",ROLLER_STATOR_LIMIT);
        DogLog.log("Intake/intake_max_current", DEPLOY_STATOR_LIMIT);
    }

    /**
     * Moves the intake up to the desired percentage of the fully up position and
     * then back down to toss the fuel into the spindexer.
     */
    public Command agitate() {
        return new ConditionalCommand(
            new SequentialCommandGroup(
                new InstantCommand(() -> deployTo(RETRACTED_ROTATIONS * INTAKE_TOSS_PERCENTAGE)),
                new WaitCommand(INTAKE_TOSS_INTERVAL_SECONDS),
                new InstantCommand(() -> deploy()),
                new WaitCommand(INTAKE_TOSS_INTERVAL_SECONDS)
            ), 
            new InstantCommand(),
            () -> Math.abs(rollerController.Output) < 0.1
        );
    }

    public boolean isDeployed() {
        return !MathUtil.isNear(
            deployMotor.getPosition().getValueAsDouble(), 
            RETRACTED_ROTATIONS, 
            0.2
        );
    }

    public void setRollerSpeed(double percentOfVoltage) {
        rollerMotor.setControl(rollerController.withOutput(INTAKE_ROLLER_VOLTAGE * percentOfVoltage));
    }

    private void deployTo(double rotations) {
        deployMotor.setControl(deployController.withSlot(0).withPosition(rotations));
    }

    public void deploy() {
        deployMotor.setControl(deployController.withSlot(1).withPosition(DEPLOYED_ROTATIONS));
    }

    public void retract() {
        deployMotor.setControl(deployController.withSlot(0).withPosition(RETRACTED_ROTATIONS));
    }

    // AUTO COMMANDS

    public Command intakeCommand() {
        return runOnce(() -> setRollerSpeed(MAX_ROLLER_SPEED));
    }

    public Command stopCommand() {
        return runOnce(() -> setRollerSpeed(0));
    }

    public Command deployCommand() {
        return runOnce(() -> deploy());
    }

    public Command retractCommand() {
        return runOnce(() -> retract());
    }

    public Command startIntaking() {
        return runOnce(() -> {
            deploy();
            setRollerSpeed(MAX_ROLLER_SPEED);
        });
    }

    @Override
    public void periodic() {
        DogLog.log("Intake/roller_speed", rollerMotor.get());
        DogLogUtil.logDouble("Intake/roller_current", rollerMotor.getSupplyCurrent().getValueAsDouble());
        DogLogUtil.logDouble("Intake/roller_stator_current", rollerMotor.getStatorCurrent().getValueAsDouble());
        DogLogUtil.logDouble("Intake/roller_temperature_C", rollerMotor.getDeviceTemp().getValueAsDouble());

        DogLogUtil.logDouble("Intake/intake_rotations", deployMotor.getPosition().getValueAsDouble());
        DogLogUtil.logDouble("Intake/intake_current", deployMotor.getSupplyCurrent().getValueAsDouble());
        DogLogUtil.logDouble("Intake/intake_stator_current", deployMotor.getStatorCurrent().getValueAsDouble());
        DogLogUtil.logDouble("Intake/intake_temperature_C", deployMotor.getDeviceTemp().getValueAsDouble());
    }
}
