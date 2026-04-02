package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.DogLogUtil;

public class TransferSubsystem extends SubsystemBase {
    private final TalonFX spindexerMotor;
    private final TalonFX feederMotor;

    private final TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration feederConfig = new TalonFXConfiguration();

    private final VoltageOut spindexerController = new VoltageOut(0);
    private final VoltageOut feederController = new VoltageOut(0);

    private final double SPINNING_VOLTAGE = 12;
    private final double FEEDING_VOLTAGE = 12;

    private final int SPINDEXER_MOTOR_ID = 5;
    private final int FEEDER_MOTOR_ID = 6;

    private final int SPINDEXER_STATOR_LIMIT = 50;
    private final int FEEDER_STATOR_LIMIT = 50;

    private static final InterpolatingDoubleTreeMap distanceToVoltage = new InterpolatingDoubleTreeMap() {{
        put(1.0, 6.0);
        put(2.5, 9.0);
        put(4.0, 12.0);
    }};
    
    private long lastMs = 0;

    public TransferSubsystem() {
        spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID);
        feederMotor = new TalonFX(FEEDER_MOTOR_ID);

        spindexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        spindexerConfig.CurrentLimits.StatorCurrentLimit = SPINDEXER_STATOR_LIMIT;

        feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        feederConfig.CurrentLimits.StatorCurrentLimit = FEEDER_STATOR_LIMIT;

        spindexerMotor.getConfigurator().apply(spindexerConfig);
        feederMotor.getConfigurator().apply(feederConfig);
    }

    public void spin(double metersToTarget) {
        spindexerMotor.setControl(spindexerController.withOutput(distanceToVoltage.get(metersToTarget)));
    }

    public void stopSpinning() {
        spindexerMotor.setControl(spindexerController.withOutput(0));
    }

    public void reverseFeed() {
        feederMotor.setControl(feederController.withOutput(-FEEDING_VOLTAGE));
    }

    public void feed(double metersToTarget) {
        feederMotor.setControl(feederController.withOutput(distanceToVoltage.get(metersToTarget)));
    }

    public void stopFeeding() {
        feederMotor.setControl(feederController.withOutput(0));
    }

    @Override
    public void periodic() {
        logMotorData();
    }

    private void logMotorData() {
        long currentMs = System.currentTimeMillis();
        if (currentMs - lastMs >= DogLogUtil.MOTOR_LOGGING_INTERVAL_MS) {
            BaseStatusSignal.refreshAll(
                spindexerMotor.getSupplyCurrent(), spindexerMotor.getStatorCurrent(), spindexerMotor.getDeviceTemp(),
                feederMotor.getSupplyCurrent(), feederMotor.getStatorCurrent(), feederMotor.getDeviceTemp()
            );

            lastMs = currentMs;

            DogLogUtil.logDouble("Transfer/spindexer_current", spindexerMotor.getSupplyCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Transfer/spindexer_stator_current", spindexerMotor.getStatorCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Transfer/spindexer_temperature_C", spindexerMotor.getDeviceTemp(false).getValueAsDouble());

            DogLogUtil.logDouble("Transfer/feeder_current", feederMotor.getSupplyCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Transfer/feeder_stator_current", feederMotor.getStatorCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Transfer/feeder_temperature_C", feederMotor.getDeviceTemp(false).getValueAsDouble());
        }
    }
}
