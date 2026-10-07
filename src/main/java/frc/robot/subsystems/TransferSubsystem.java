package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggingUtility;

public class TransferSubsystem extends SubsystemBase {
    private final TalonFX spindexerMotor;
    private final TalonFX feederMotor;

    private final TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration feederConfig = new TalonFXConfiguration();

    private final StatusSignal<Current> spindexerSupplyCurrent;
    private final StatusSignal<Current> spindexerStatorCurrent;
    private final StatusSignal<Temperature> spindexerDeviceTemp;

    private final StatusSignal<Current> feederSupplyCurrent;
    private final StatusSignal<Current> feederStatorCurrent;
    private final StatusSignal<Temperature> feederDeviceTemp;

    private final VoltageOut spindexerController = new VoltageOut(0);
    private final VoltageOut feederController = new VoltageOut(0);
    
    //TODO: Determine if necessary after changing shooting trajectory to be lower
    private final boolean USE_INTERPOLATION = false;

    private final double SPINNING_VOLTAGE = 12;
    private final double FEEDING_VOLTAGE = 12;

    private final int SPINDEXER_MOTOR_ID = 5;
    private final int FEEDER_MOTOR_ID = 6;

    private final int SPINDEXER_SUPPLY_LIMIT = 40;
    private final int FEEDER_SUPPLY_LIMIT = 60;

    private final int SPINDEXER_STATOR_LIMIT = 45;
    private final int FEEDER_STATOR_LIMIT = 50;
    
    private final InterpolatingDoubleTreeMap distanceToVoltageMap = new InterpolatingDoubleTreeMap() {{
        put(2.0, 4.0);
        put(3.0, 8.0);
        put(4.0, 12.0);
        put(6.0, 12.0);
    }};
    
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

        spindexerSupplyCurrent = spindexerMotor.getSupplyCurrent();
        spindexerStatorCurrent = spindexerMotor.getStatorCurrent();
        spindexerDeviceTemp = spindexerMotor.getDeviceTemp();

        feederSupplyCurrent = feederMotor.getSupplyCurrent();
        feederStatorCurrent = feederMotor.getStatorCurrent();
        feederDeviceTemp = feederMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            10.0,
            spindexerSupplyCurrent, spindexerStatorCurrent, spindexerDeviceTemp,
            feederSupplyCurrent, feederStatorCurrent, feederDeviceTemp
        );
    }

    public void spin(double metersToTarget) {
        if (USE_INTERPOLATION)
            spinAtVoltage(distanceToVoltageMap.get(metersToTarget));
        else
            spinAtVoltage(SPINNING_VOLTAGE);
    }

    public void reverseSpin() {
        spinAtVoltage(-SPINNING_VOLTAGE);
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

    public void feed() {
        feederMotor.setControl(
            feederController.withOutput(FEEDING_VOLTAGE)
        );
    }

    public void stopFeeding() {
        feederMotor.setControl(feederController.withOutput(0));
    }

    @Override
    public void periodic() {
        if (LoggingUtility.LOG_TRANSFER && LoggingUtility.updateLowPriorityLogs()) {
            BaseStatusSignal.refreshAll(
                spindexerSupplyCurrent, spindexerStatorCurrent, spindexerDeviceTemp,
                feederSupplyCurrent, feederStatorCurrent, feederDeviceTemp
            );

            LoggingUtility.logDouble("Transfer/spindexer_current", spindexerSupplyCurrent.getValueAsDouble());
            LoggingUtility.logDouble("Transfer/spindexer_stator_current", spindexerStatorCurrent.getValueAsDouble());
            LoggingUtility.logDouble("Transfer/spindexer_temperature_C", spindexerDeviceTemp.getValueAsDouble());

            LoggingUtility.logDouble("Transfer/feeder_current", feederSupplyCurrent.getValueAsDouble());
            LoggingUtility.logDouble("Transfer/feeder_stator_current", feederStatorCurrent.getValueAsDouble());
            LoggingUtility.logDouble("Transfer/feeder_temperature_C", feederDeviceTemp.getValueAsDouble());
        }
    }
}
