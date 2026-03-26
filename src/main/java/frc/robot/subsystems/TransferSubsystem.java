package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.DogLogUtil;

public class TransferSubsystem extends SubsystemBase {
    private final TalonFX spindexerMotor;
    private final TalonFX feederMotor;

    private final TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration feederConfig = new TalonFXConfiguration();

    private final VoltageOut spindexerController = new VoltageOut(0)
        .withEnableFOC(true);
    private final VoltageOut feederController = new VoltageOut(0)
        .withEnableFOC(true);

    private final CoastOut coast = new CoastOut();

    private final double SPINNING_VOLTAGE = 12; //TODO: Possibly slow down to improve feed rate
    private final double FEEDING_VOLTAGE = 12;

    private final int SPINDEXER_MOTOR_ID = 5;
    private final int FEEDER_MOTOR_ID = 6;

    private final int SPINDEXER_STATOR_LIMIT = 60;
    private final int FEEDER_STATOR_LIMIT = 50;
    
    private long lastStatusRefreshMs = 0;

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

        spindexerMotor.setControl(coast);
        feederMotor.setControl(coast);
    }

    public void spin() {
        spindexerMotor.setControl(spindexerController.withOutput(SPINNING_VOLTAGE));
    }

    public void stopSpinning() {
        spindexerMotor.setControl(spindexerController.withOutput(0));
    }

    public void reverseFeed() {
        feederMotor.setControl(feederController.withOutput(-FEEDING_VOLTAGE));
    }

    public void feed() {
        feederMotor.setControl(feederController.withOutput(FEEDING_VOLTAGE));
    }

    public void stopFeeding() {
        feederMotor.setControl(feederController.withOutput(0));
    }

    @Override
    public void periodic() {
        logMotorData();
    }

    private void logMotorData() {
        // Throttle status refreshes and only log currents/temps when we've refreshed the cached values.
        long nowMs = System.currentTimeMillis();
        if (nowMs - lastStatusRefreshMs >= DogLogUtil.MOTOR_LOGGING_INTERVAL_MS) {
            BaseStatusSignal.refreshAll(
                spindexerMotor.getSupplyCurrent(), spindexerMotor.getStatorCurrent(), spindexerMotor.getDeviceTemp(),
                feederMotor.getSupplyCurrent(), feederMotor.getStatorCurrent(), feederMotor.getDeviceTemp()
            );

            lastStatusRefreshMs = nowMs;

            DogLogUtil.logDouble("Transfer/spindexer_current", spindexerMotor.getSupplyCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Transfer/spindexer_stator_current", spindexerMotor.getStatorCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Transfer/spindexer_temperature_C", spindexerMotor.getDeviceTemp(false).getValueAsDouble());

            DogLogUtil.logDouble("Transfer/feeder_current", feederMotor.getSupplyCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Transfer/feeder_stator_current", feederMotor.getStatorCurrent(false).getValueAsDouble());
            DogLogUtil.logDouble("Transfer/feeder_temperature_C", feederMotor.getDeviceTemp(false).getValueAsDouble());
        }
    }
}
