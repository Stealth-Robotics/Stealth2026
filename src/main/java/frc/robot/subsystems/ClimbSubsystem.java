package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    // private final TalonFX climbMotor1;
    // private final TalonFX climbMotor2;

    // private final TalonFXConfiguration climbConfig = new TalonFXConfiguration();

    // //TODO: Find CAN IDs
    // private final int CLIMB_MOTOR_1_ID = 0;
    // private final int CLIMB_MOTOR_2_ID = 0;

    // public ClimbSubsystem() {
    //     climbMotor1 = new TalonFX(CLIMB_MOTOR_1_ID);
    //     climbMotor2 = new TalonFX(CLIMB_MOTOR_2_ID);

    //     climbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    //     //Coast or brake? We want brake for when we need to slowly descend after a match
    //     climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    //     climbMotor1.getConfigurator().apply(climbConfig);
    //     climbMotor2.getConfigurator().apply(climbConfig);

    //     climbMotor2.setControl(new Follower(CLIMB_MOTOR_1_ID, MotorAlignmentValue.Opposed));
    // }

    // @Override
    // public void periodic() {
    // }
}
