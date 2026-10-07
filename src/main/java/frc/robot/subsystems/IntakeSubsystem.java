package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.util.LoggingUtility;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX leftRollerMotor;
    private final TalonFX rightRollerMotor;

    private final TalonFX deployMotor;

    private final StatusSignal<AngularVelocity> leftRollerVelocity;
    private final StatusSignal<Current> leftRollerSupplyCurrent;
    private final StatusSignal<Current> leftRollerStatorCurrent;
    private final StatusSignal<Temperature> leftRollerTemp;

    private final StatusSignal<AngularVelocity> rightRollerVelocity;
    private final StatusSignal<Current> rightRollerSupplyCurrent;
    private final StatusSignal<Current> rightRollerStatorCurrent;
    private final StatusSignal<Temperature> rightRollerTemp;

    private final StatusSignal<Current> deploySupplyCurrent;
    private final StatusSignal<Current> deployStatorCurrent;
    private final StatusSignal<Temperature> deployTemp;

    private final CANcoder deployEncoder;
    private final CANcoderConfiguration deployEncoderConfig = new CANcoderConfiguration();

    private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration deployConfig = new TalonFXConfiguration();

    private final MotionMagicVoltage deployController = new MotionMagicVoltage(0);
    private final VoltageOut rollerController = new VoltageOut(0);

    private double targetRollerSpeed = 0;

    private final double INTAKE_ROLLER_VOLTAGE = 12;
    private final double MAX_ROLLER_SPEED = 0.75;

    private final double DEPLOY_ENCODER_ZERO_OFFSET = -0.3984375;

    private final double DEPLOY_ENCODER_TO_MECHANISM_RATIO = 1.0;
    private final double DEPLOY_MOTOR_TO_ENCODER_RATIO = 52.0;

    private final double DEPLOY_ENCODER_DISCONTINUTY_POINT = 0.651;
    private final double DEPLOY_POSITION_TOLERANCE = 0.05;

    private final double DEPLOYED_ROTATIONS = -0.02;
    private final double SAFE_ROTATIONS = 0.15;
    private final double FULL_AGITATE_ROTATIONS = 0.3;
    private final double RETRACTED_ROTATIONS = 0.32;

    private final double DEPLOY_kP = 40;
    private final double RETRACT_kP = 36;
    private final double FAST_kP = 70;

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

    private final double INTAKE_TOSS_INTERVAL_SECONDS = 0.25;
    private boolean isRetracting = false;
 
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
        
        leftRollerVelocity = leftRollerMotor.getVelocity();
        leftRollerSupplyCurrent = leftRollerMotor.getSupplyCurrent();
        leftRollerStatorCurrent = leftRollerMotor.getStatorCurrent();
        leftRollerTemp = leftRollerMotor.getDeviceTemp();

        rightRollerVelocity = rightRollerMotor.getVelocity();
        rightRollerSupplyCurrent = rightRollerMotor.getSupplyCurrent();
        rightRollerStatorCurrent = rightRollerMotor.getStatorCurrent();
        rightRollerTemp = rightRollerMotor.getDeviceTemp();

        deploySupplyCurrent = deployMotor.getSupplyCurrent();
        deployStatorCurrent = deployMotor.getStatorCurrent();
        deployTemp = deployMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            10.0,
            leftRollerVelocity, leftRollerSupplyCurrent, leftRollerStatorCurrent, leftRollerTemp,
            rightRollerVelocity, rightRollerSupplyCurrent, rightRollerStatorCurrent, rightRollerTemp,
            deploySupplyCurrent, deployStatorCurrent, deployTemp
        );
    }

    public Command quickAgitate(DoubleSupplier magnitude) {
        var command = new SequentialCommandGroup(
            new InstantCommand(() -> setRollerSpeed(0.25)),
            new InstantCommand(() -> moveFastTo(magnitude.getAsDouble() * RETRACTED_ROTATIONS)),
            new WaitCommand(INTAKE_TOSS_INTERVAL_SECONDS),
            deployCommand(),
            stopRollers(),
            new WaitCommand(INTAKE_TOSS_INTERVAL_SECONDS)
        ).finallyDo(() -> {
            deploy();
            setRollerSpeed(0);
        });
        
        command.addRequirements(this);
        return command;
    }

    public Command fullAgitate() {
        final double ROTATION_INCREMENT = 0.02;

        var command =
            new RunCommand(() -> {
                setRollerSpeed(0.25);

                deployMotor.setControl(deployController.withSlot(1).withPosition(MathUtil.clamp(
                    deployController.getPositionMeasure().magnitude() + ROTATION_INCREMENT,
                    DEPLOYED_ROTATIONS,
                    FULL_AGITATE_ROTATIONS
                )));
            })
            .finallyDo(() -> {
                deploy();
                setRollerSpeed(0);
            });

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

    public void deploy() {
        isRetracting = false;
        deployMotor.setControl(deployController.withSlot(0).withPosition(DEPLOYED_ROTATIONS));
    }

    public void retract() {
        isRetracting = true;
        deployMotor.setControl(deployController.withSlot(1).withPosition(RETRACTED_ROTATIONS));
    }

    public void safe() {
        isRetracting = false;
        deployMotor.setControl(deployController.withSlot(1).withPosition(SAFE_ROTATIONS));
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

        LoggingUtility.logDoubleForceNT("Intake/deploy_rotations", deployMotor.getPosition().getValueAsDouble());

        if (LoggingUtility.LOG_INTAKE && LoggingUtility.updateLowPriorityLogs()) {
            BaseStatusSignal.refreshAll(
                leftRollerVelocity, leftRollerSupplyCurrent, leftRollerStatorCurrent, leftRollerTemp,
                rightRollerVelocity, rightRollerSupplyCurrent, rightRollerStatorCurrent, rightRollerTemp,
                deploySupplyCurrent, deployStatorCurrent, deployTemp
            );

            LoggingUtility.logDouble("Intake/left_roller_rpm", leftRollerVelocity.getValueAsDouble() * 60.0);
            LoggingUtility.logDouble("Intake/left_roller_supply_current", leftRollerSupplyCurrent.getValueAsDouble());
            LoggingUtility.logDouble("Intake/left_roller_stator_current", leftRollerStatorCurrent.getValueAsDouble());
            LoggingUtility.logDouble("Intake/left_roller_temperature_C", leftRollerTemp.getValueAsDouble());

            LoggingUtility.logDouble("Intake/right_roller_rpm", rightRollerVelocity.getValueAsDouble() * 60.0);
            LoggingUtility.logDouble("Intake/right_roller_supply_current", rightRollerSupplyCurrent.getValueAsDouble());
            LoggingUtility.logDouble("Intake/right_roller_stator_current", rightRollerStatorCurrent.getValueAsDouble());
            LoggingUtility.logDouble("Intake/right_roller_temperature_C", rightRollerTemp.getValueAsDouble());

            LoggingUtility.logDouble("Intake/intake_supply_current", deploySupplyCurrent.getValueAsDouble());
            LoggingUtility.logDouble("Intake/intake_stator_current", deployStatorCurrent.getValueAsDouble());
            LoggingUtility.logDouble("Intake/intake_temperature_C", deployTemp.getValueAsDouble());
        }
    }
}
