package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.DogLogUtil;

public class TransferSubsystem extends SubsystemBase {
    private final TalonFX spindexerMotor;
    private final TalonFX feederMotor;

    private final TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration feederConfig = new TalonFXConfiguration();

    private final VoltageOut spindexerController = new VoltageOut(0);
    private final VoltageOut feederController = new VoltageOut(0);

    private final double FEEDING_VOLTAGE = 12;

    private final int SPINDEXER_MOTOR_ID = 5;
    private final int FEEDER_MOTOR_ID = 6;

    private final int SPINDEXER_SUPPLY_LIMIT = 40;
    private final int FEEDER_SUPPLY_LIMIT = 60;

    private final int SPINDEXER_STATOR_LIMIT = 45;
    private final int FEEDER_STATOR_LIMIT = 45;

    private static final InterpolatingDoubleTreeMap distanceToVoltageMap = new InterpolatingDoubleTreeMap() {{
        put(2.0, 5.0);
        put(3.0, 7.0);
        put(3.5, 12.0);
    }};
    
    private long lastMs = 0;

    public TransferSubsystem() {
        spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID);
        feederMotor = new TalonFX(FEEDER_MOTOR_ID);

        spindexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        spindexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        spindexerConfig.CurrentLimits.SupplyCurrentLimit = SPINDEXER_SUPPLY_LIMIT;
        spindexerConfig.CurrentLimits.StatorCurrentLimit = SPINDEXER_STATOR_LIMIT;
        
        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        feederConfig.CurrentLimits.SupplyCurrentLimit = FEEDER_SUPPLY_LIMIT;
        feederConfig.CurrentLimits.StatorCurrentLimit = FEEDER_STATOR_LIMIT;

        spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        spindexerConfig.CurrentLimits.StatorCurrentLimit = 0;

        feederConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        feederConfig.CurrentLimits.StatorCurrentLimit = 0;

        spindexerMotor.getConfigurator().apply(spindexerConfig);
        feederMotor.getConfigurator().apply(feederConfig);
    }

    public void spin(double metersToTarget) {
        spinAtVoltage(distanceToVoltageMap.get(metersToTarget));
    }

    private void spinAtVoltage(double voltage) {
        spindexerMotor.setControl(
            spindexerController.withOutput(voltage)
        );
    }

    public void stopSpinning() {
        spinAtVoltage(0);
    }

    public void reverseFeed() {
        feederMotor.setControl(feederController.withOutput(-FEEDING_VOLTAGE));
    }

    public void feed(double metersToTarget) {
        feederMotor.setControl(
            feederController.withOutput(distanceToVoltageMap.get(metersToTarget))
        );
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
