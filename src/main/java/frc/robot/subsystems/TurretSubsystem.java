package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final MotionMagicVoltage turretController = new MotionMagicVoltage(0);
    private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();

    private final double kP = 0.0;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private final int TURRET_MOTOR_CAN_ID = 0;

    public TurretSubsystem() {
        turretMotor = new TalonFX(TURRET_MOTOR_CAN_ID);


        turretMotor.getConfigurator().apply(turretConfig);
    }
}
