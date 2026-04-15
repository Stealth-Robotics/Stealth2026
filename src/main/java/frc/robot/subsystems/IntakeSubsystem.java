package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.util.DogLogUtil;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX leftRollerMotor;
    private final TalonFX rightRollerMotor;

    private final TalonFX deployMotor;

    private final CANcoder deployEncoder;
    private final CANcoderConfiguration deployEncoderConfig = new CANcoderConfiguration();

    private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration deployConfig = new TalonFXConfiguration();

    private final MotionMagicVoltage deployController = new MotionMagicVoltage(0);
    private final VoltageOut rollerController = new VoltageOut(0);

    private double targetRollerSpeed = 0;

    private final double INTAKE_ROLLER_VOLTAGE = 12;
    private final double MAX_ROLLER_SPEED = 0.8;

    private final double DEPLOY_ENCODER_ZERO_OFFSET = -0.401123046875;

    private final double DEPLOY_ENCODER_TO_MECHANISM_RATIO = 1.0;
    private final double DEPLOY_MOTOR_TO_ENCODER_RATIO = 52.0;

    private final double DEPLOY_ENCODER_DISCONTINUTY_POINT = 0.651;
    private final double DEPLOY_POSITION_TOLERANCE = 0.05;

    private final double DEPLOYED_ROTATIONS = 0.0;
    private final double SAFE_ROTATIONS = 0.08;
    private final double RETRACTED_ROTATIONS = 0.305;

    private final double DEPLOY_kP = 35;
    private final double RETRACT_kP = 30;
    private final double FAST_kP = 40;

    private final double DEPLOY_kACCEL = 20;
    private final double DEPLOY_kVELO = 30;

    private final int ROLLER_MOTOR_LEFT_ID = 16;
    private final int ROLLER_MOTOR_RIGHT_ID = 26;
    private final int DEPLOY_MOTOR_ID = 17;
    private final int DEPLOY_ENCODER_ID = 18;

    private final int DEPLOY_STATOR_LIMIT = 50;
    private final int ROLLER_STATOR_LIMIT = 80;

    private final int ROLLER_SUPPLY_LIMIT = 35;
    private final int DEPLOY_SUPPLY_LIMIT = 30;

    private final double INTAKE_TOSS_INTERVAL_SECONDS = 0.3;
    private boolean isRetracting = false;

    private long lastMs = 0;
 
    public IntakeSubsystem() {
        leftRollerMotor = new TalonFX(ROLLER_MOTOR_LEFT_ID);
        rightRollerMotor = new TalonFX(ROLLER_MOTOR_RIGHT_ID);
        deployMotor = new TalonFX(DEPLOY_MOTOR_ID);
        deployEncoder = new CANcoder(DEPLOY_ENCODER_ID);

        //Roller motor config
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rollerConfig.CurrentLimits.StatorCurrentLimit = ROLLER_STATOR_LIMIT;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        rollerConfig.CurrentLimits.SupplyCurrentLimit = ROLLER_SUPPLY_LIMIT;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftRollerMotor.getConfigurator().apply(rollerConfig);
        rightRollerMotor.getConfigurator().apply(rollerConfig);
        rightRollerMotor.setControl(new Follower(ROLLER_MOTOR_LEFT_ID, MotorAlignmentValue.Opposed));

        //CANCoder config
        deployEncoderConfig.MagnetSensor.MagnetOffset = DEPLOY_ENCODER_ZERO_OFFSET;
        deployEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        deployEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = DEPLOY_ENCODER_DISCONTINUTY_POINT;

        deployEncoder.getConfigurator().apply(deployEncoderConfig);
        
        //Deploy motor config
        deployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        deployConfig.MotorOutput.DutyCycleNeutralDeadband = DEPLOY_POSITION_TOLERANCE / 2.0;

        deployConfig.Feedback.FeedbackRemoteSensorID = deployEncoder.getDeviceID();
        deployConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        deployConfig.Feedback.SensorToMechanismRatio = DEPLOY_ENCODER_TO_MECHANISM_RATIO;
        deployConfig.Feedback.RotorToSensorRatio = DEPLOY_MOTOR_TO_ENCODER_RATIO;

        deployConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        deployConfig.CurrentLimits.StatorCurrentLimit = DEPLOY_STATOR_LIMIT;

        deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        deployConfig.CurrentLimits.SupplyCurrentLimit = DEPLOY_SUPPLY_LIMIT;

        deployConfig.Slot0.kP = DEPLOY_kP;
        deployConfig.Slot1.kP = RETRACT_kP;
        deployConfig.Slot2.kP = FAST_kP;
        
        deployConfig.MotionMagic.MotionMagicAcceleration = DEPLOY_kACCEL;
        deployConfig.MotionMagic.MotionMagicCruiseVelocity = DEPLOY_kVELO;

        deployMotor.getConfigurator().apply(deployConfig);
        deployMotor.setControl(deployController.withSlot(0).withPosition(deployMotor.getPosition().getValue()));
    }

    public Command partialAgitate(DoubleSupplier magnitude) {
        var command = new SequentialCommandGroup(
            new InstantCommand(() -> setRollerSpeed(MAX_ROLLER_SPEED * 0.5)),
            new InstantCommand(() -> moveFastTo(magnitude.getAsDouble() * RETRACTED_ROTATIONS)),
            new WaitUntilCommand(()-> isAtPosition(magnitude.getAsDouble() * RETRACTED_ROTATIONS)).withTimeout(0.2),
            new WaitCommand(INTAKE_TOSS_INTERVAL_SECONDS),
            new InstantCommand(() -> deploy()),
            new InstantCommand(() -> setRollerSpeed(0))
        );
        
        command.addRequirements(this);
        return command;
    }

    public Command fullAgitate() {
        double tossPercentage = RETRACTED_ROTATIONS * 0.65;
        
        var command = new SequentialCommandGroup(
            new InstantCommand(() -> setRollerSpeed(MAX_ROLLER_SPEED * 0.5)),
            new InstantCommand(() -> moveFastTo(tossPercentage)),
            new WaitUntilCommand(() -> isAtPosition(tossPercentage)).withTimeout(0.5),
            new InstantCommand(() -> setRollerSpeed(0))
        )
        .andThen(new RunCommand(() -> moveFastTo(deployController.getPositionMeasure().magnitude() + 0.008)))
        .finallyDo(() -> deploy());

        command.addRequirements(this);
        return command;
    }

    public void setRollerSpeed(double speed) {
        targetRollerSpeed = speed;
    }

    private void moveFastTo(double rotations) {
        deployMotor.setControl(deployController.withPosition(MathUtil.clamp(rotations, DEPLOYED_ROTATIONS, RETRACTED_ROTATIONS)).withSlot(2));
    }

    public boolean isAtPosition(double rotations) {
        double curPos = deployMotor.getPosition().getValueAsDouble();
        double error = Math.abs(curPos - rotations);
        
        return error <= DEPLOY_POSITION_TOLERANCE;
    }

    public boolean isRetracting() {
        return isRetracting;
    }

    public boolean isDeployed() {
        return isAtPosition(DEPLOYED_ROTATIONS);
    }

    public boolean isSafe() {
        return isAtPosition(SAFE_ROTATIONS);
    }

    public void safe() {
        isRetracting = false;
        deployMotor.setControl(deployController.withSlot(1).withPosition(SAFE_ROTATIONS));
    }

    public void deploy() {
        isRetracting = false;
        deployMotor.setControl(deployController.withSlot(0).withPosition(DEPLOYED_ROTATIONS));
    }

    public void retract() {
        isRetracting = true;
        deployMotor.setControl(deployController.withSlot(1).withPosition(RETRACTED_ROTATIONS));
    }

    // AUTO COMMANDS

    public Command startRollers() {
        return runOnce(() -> setRollerSpeed(MAX_ROLLER_SPEED));
    }

    public Command stopRollers() {
        return runOnce(() -> setRollerSpeed(0));
    }

    public Command deployCommand() {
        return runOnce(() -> deploy());
    }

    public Command retractCommand() {
        return runOnce(() -> retract());
    }

    public Command safeCommand() {
        return runOnce(() -> safe());
    }

    @Override
    public void periodic() {
        leftRollerMotor.setControl(rollerController.withOutput(
            INTAKE_ROLLER_VOLTAGE * MathUtil.clamp(targetRollerSpeed, -MAX_ROLLER_SPEED, MAX_ROLLER_SPEED)
        ));

        DogLogUtil.logDoubleForceNT("Intake/deploy_rotations", deployMotor.getPosition().getValueAsDouble());

        logMotorData();
    }

    private void logMotorData() {
        long currentMs = System.currentTimeMillis();

        if (currentMs - lastMs >= DogLogUtil.MOTOR_LOGGING_INTERVAL_MS) {
            BaseStatusSignal.refreshAll(
                leftRollerMotor.getSupplyCurrent(), leftRollerMotor.getStatorCurrent(), leftRollerMotor.getDeviceTemp(),
                rightRollerMotor.getSupplyCurrent(), rightRollerMotor.getStatorCurrent(), rightRollerMotor.getDeviceTemp(),
                deployMotor.getSupplyCurrent(), deployMotor.getStatorCurrent(), deployMotor.getDeviceTemp(),
                leftRollerMotor.getVelocity(), rightRollerMotor.getVelocity()
            );

            lastMs = currentMs;

            DogLogUtil.logDouble("Intake/left_roller_rpm", leftRollerMotor.getVelocity(false).getValueAsDouble() * 60.0);
            DogLogUtil.logDouble("Intake/left_roller_supply_current", leftRollerMotor.getSupplyCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Intake/left_roller_stator_current", leftRollerMotor.getStatorCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Intake/left_roller_temperature_C", leftRollerMotor.getDeviceTemp(false).getValueAsDouble());

            DogLogUtil.logDouble("Intake/right_roller_rpm", rightRollerMotor.getVelocity(false).getValueAsDouble() * 60.0);
            DogLogUtil.logDouble("Intake/right_roller_supply_current", rightRollerMotor.getSupplyCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Intake/right_roller_stator_current", rightRollerMotor.getStatorCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Intake/right_roller_temperature_C", rightRollerMotor.getDeviceTemp(false).getValueAsDouble());

            DogLogUtil.logDouble("Intake/intake_supply_current", deployMotor.getSupplyCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Intake/intake_stator_current", deployMotor.getStatorCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Intake/intake_temperature_C", deployMotor.getDeviceTemp(false).getValueAsDouble());
        }
    }
}
