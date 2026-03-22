package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
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

    private final double INTAKE_ROLLER_VOLTAGE = 12;
    private final double MAX_ROLLER_SPEED = 0.8;

    private final double DEPLOY_ENCODER_ZERO_OFFSET = -0.402588;

    private final double DEPLOY_ENCODER_TO_MECHANISM_RATIO = 1.0;
    private final double DEPLOY_MOTOR_TO_ENCODER_RATIO = 52.0;

    private final double TURRET_ENCODER_DISCONTINUTY_POINT = 0.651;

    private final double DEPLOYED_ROTATIONS = 0;
    private final double RETRACTED_ROTATIONS = 0.308;

    private final double DEPLOY_kP = 50;
    private final double RETRACT_kP = 50;
    private final double DEPLOY_kI = 0;
    private final double RETRACT_kI = 0.09;
    private final double DEPLOY_kACCEL = 20;
    private final double DEPLOY_kVELO = 60;

    private final int ROLLER_MOTOR_ID = 16;
    private final int DEPLOY_MOTOR_ID = 17;
    private final int DEPLOY_ENCODER_ID = 18;

    private final int DEPLOY_STATOR_LIMIT = 20;
    private final int ROLLER_STATOR_LIMIT = 90;

    private final double INTAKE_TOSS_INTERVAL_SECONDS = 0.35;
    private final double INTAKE_TOSS_PERCENTAGE = 1;

    private boolean isIntaking = false;

    // Throttle refreshes to 10 Hz
    private long lastStatusRefreshMs = 0;

    private boolean first = true;
 
    public IntakeSubsystem() {
        rollerMotor = new TalonFX(ROLLER_MOTOR_ID);
        deployMotor = new TalonFX(DEPLOY_MOTOR_ID);
        deployEncoder = new CANcoder(DEPLOY_ENCODER_ID);

        //Roller motor config
        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // rollerConfig.CurrentLimits.StatorCurrentLimit = ROLLER_STATOR_LIMIT;
        // rollerConfig.CurrentLimits.SupplyCurrentLimit = 30;
        // rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

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

    public void isIntaking(boolean stateUpdate) {
        this.isIntaking = stateUpdate;
    }

    /**
     * Moves the intake up to the desired percentage of the fully up position and
     * then back down to toss the fuel into the spindexer.
     */
    public Command agitate() {
        return new ConditionalCommand(
            new SequentialCommandGroup(
                new InstantCommand(() -> setRollerSpeed(MAX_ROLLER_SPEED)),
                new InstantCommand(() -> retract(), this),
                new WaitCommand(INTAKE_TOSS_INTERVAL_SECONDS),
                new InstantCommand(() -> deploy(), this),
                new WaitCommand(INTAKE_TOSS_INTERVAL_SECONDS / 0.25),
                new InstantCommand(() -> setRollerSpeed(0))
            ),
            new InstantCommand(),
            () -> !isIntaking
        );
        // return new SequentialCommandGroup(
        //     new InstantCommand(() -> setRollerSpeed(MAX_ROLLER_SPEED)),
        //     new InstantCommand(() -> deployTo(RETRACTED_ROTATIONS * INTAKE_TOSS_PERCENTAGE), this),
        //     new WaitCommand(INTAKE_TOSS_INTERVAL_SECONDS),
        //     new InstantCommand(() -> deploy(), this),
        //     new WaitCommand(INTAKE_TOSS_INTERVAL_SECONDS / 0.5),
        //     new InstantCommand(() -> setRollerSpeed(0))
        // );
    }

    public void setRollerSpeed(double percentOfVoltage) {
        // rollerMotor.setControl(rollerController.withOutput(INTAKE_ROLLER_VOLTAGE * percentOfVoltage));
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
        return run(() -> setRollerSpeed(MAX_ROLLER_SPEED));
    }

    public Command stopCommand() {
        return run(() -> setRollerSpeed(0));
    }

    public Command deployCommand() {
        return runOnce(() -> deploy());
    }

    public Command retractCommand() {
        return runOnce(() -> retract());
    }

    @Override
    public void periodic() {
        if (first) {
            rollerMotor.set(-1.0);
            first = false;
        }

        DogLog.log("Intake/roller_speed", rollerMotor.get());
        DogLogUtil.logDouble("Intake/intake_rotations", deployMotor.getPosition().getValueAsDouble());
        logMotorData();
    }

    private void logMotorData() {
        // Throttle status refreshes and only log currents/temps when we've refreshed the cached values.
        long nowMs = System.currentTimeMillis();
        if (nowMs - lastStatusRefreshMs >= DogLogUtil.MOTOR_LOGGING_INTERVAL_MS) {
            BaseStatusSignal.refreshAll(
                rollerMotor.getSupplyCurrent(), rollerMotor.getStatorCurrent(), rollerMotor.getDeviceTemp(),
                deployMotor.getSupplyCurrent(), deployMotor.getStatorCurrent(), deployMotor.getDeviceTemp()
            );

            lastStatusRefreshMs = nowMs;

            DogLogUtil.logDouble("Intake/roller_current", rollerMotor.getSupplyCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Intake/roller_stator_current", rollerMotor.getStatorCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Intake/roller_supply_current", rollerMotor.getSupplyCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Intake/roller_temperature_C", rollerMotor.getDeviceTemp(false).getValueAsDouble());
            DogLogUtil.logDouble("Intake/roller_rpm", rollerMotor.getVelocity().getValueAsDouble() * 60.0);

            DogLogUtil.logDouble("Intake/intake_current", deployMotor.getSupplyCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Intake/intake_stator_current", deployMotor.getStatorCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Intake/intake_temperature_C", deployMotor.getDeviceTemp(false).getValueAsDouble());
        }
    
    }
}
