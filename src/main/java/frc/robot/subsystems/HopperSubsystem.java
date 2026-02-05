package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase{
    private final TalonFX feederMotor;
    private final TalonFXConfiguration feederConfig = new TalonFXConfiguration();

    //TODO: Find correct CAN IDs
    private final int FEEDER_MOTOR_ID = 0;

    public HopperSubsystem() {
        feederMotor = new TalonFX(FEEDER_MOTOR_ID);

        feederMotor.getConfigurator().apply(feederConfig);
    }
}
