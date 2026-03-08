package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    private final double SPINNING_VOLTAGE = 10;
    private final double FEEDING_VOLTAGE = 12;

    private final int SPINDEXER_MOTOR_ID = 5;
    private final int FEEDER_MOTOR_ID = 6;

    public TransferSubsystem() {
        spindexerMotor = new TalonFX(SPINDEXER_MOTOR_ID);
        feederMotor = new TalonFX(FEEDER_MOTOR_ID);

        spindexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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
}
