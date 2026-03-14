package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.DogLogUtil;

public class ClimbSubsystem extends SubsystemBase {
    private final TalonFX climbMotor1;
    private final TalonFX climbMotor2;
    // private final Servo clipServo;

    private final TalonFXConfiguration climbConfig = new TalonFXConfiguration();
    private final MotionMagicVoltage climbController = new MotionMagicVoltage(0);

    //TODO: Find CAN IDs
    private final int CLIMB_MOTOR_1_ID = 26;
    private final int CLIMB_MOTOR_2_ID = 28;
    // private final int CLIP_SERVO_CHANNEL = 0;

    // private final double CLIP_SERVO_IN = 0.0;
    // private final double CLIP_SERVO_OUT = 1.0;

    private final double CLIMB_IDLE_POS_INCHES = 0.0;
    private final double CLIMB_REACH_POS_INCHES = 6.25;
    private final double CLIMB_ASCENT_POS_INCHES = 0.0;

    private final double CLIMB_SENSOR_TO_MECHANISM_RATIO = 1.0;

    private final double kP = 1;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kG = 0.0;
    private final double kACCELERATION = 5;
    private final double kCRUISE_VELOCITY = 2;

    private final double CLIMB_ZERO_POS = 0.0;

    private boolean climberUp = false;

    public ClimbSubsystem() {
        climbMotor1 = new TalonFX(CLIMB_MOTOR_1_ID);
        climbMotor2 = new TalonFX(CLIMB_MOTOR_2_ID);
        // clipServo = new Servo(CLIP_SERVO_CHANNEL);

        climbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climbConfig.Feedback.SensorToMechanismRatio = CLIMB_SENSOR_TO_MECHANISM_RATIO;

        climbConfig.Slot0.kP = kP;
        climbConfig.Slot0.kI = kI;
        climbConfig.Slot0.kD = kD;
        climbConfig.Slot0.kG = kG;
        climbConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        climbConfig.MotionMagic.MotionMagicAcceleration = kACCELERATION;
        climbConfig.MotionMagic.MotionMagicCruiseVelocity = kCRUISE_VELOCITY;

        climbMotor1.getConfigurator().apply(climbConfig);
        climbMotor2.getConfigurator().apply(climbConfig);

        climbMotor2.setControl(new Follower(CLIMB_MOTOR_1_ID, MotorAlignmentValue.Aligned));
    }
    private void runToPosition(double inches) {
        climbMotor1.setControl(climbController.withSlot(0).withPosition(inches - CLIMB_ZERO_POS));
    }
    public Command reach() {
        return new InstantCommand(() -> runToPosition(CLIMB_REACH_POS_INCHES));
    }
    public Command ascend() {
        return new InstantCommand(() -> runToPosition(CLIMB_ASCENT_POS_INCHES));
    }

    public Command toggleClimb() {
        return new ConditionalCommand(ascend(), reach(), () -> climberUp).andThen(() -> climberUp = !climberUp);
    }
    public Command stow() {
        return new InstantCommand(() -> runToPosition(CLIMB_IDLE_POS_INCHES));
    }

    @Override
    public void periodic() {
        DogLogUtil.logDouble("Climb/climberPose", climbMotor1.get());
    }
}
