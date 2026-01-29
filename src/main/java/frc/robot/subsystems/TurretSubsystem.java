package frc.robot.subsystems;

import java.util.function.Supplier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldConstants;
import frc.robot.util.Pose;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final Supplier<Pose> robotPoseSupplier;

    private Pose hubPose = null;

    private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    private final MotionMagicVoltage turretController = new MotionMagicVoltage(0);

    //TODO: Find actual values from robot
    private final double TURRET_ORIGIN_OFFSET_X_INCHES = 0; // Positive values are to the right of the origin
    private final double TURRET_ORIGIN_OFFSET_Y_INCHES = 0; // Positive values are to the top of the origin

    //TODO: Tune PID/Feedforward constants
    private final double kACCELERATION = 0.0;
    private final double kCRUISE_VELOCITY = 0.0;
    private final double kP = 0.0;
    private final double kI = 0.0;
    private final double kD = 0.0;

    //TODO: Find actual values
    private final double MAX_TURRET_ROTATIONS = 0;
    private final double MIN_TURRET_ROTATIONS = 0;

    //TODO: Find correct CAN IDs
    private final int TURRET_MOTOR_ID = 0;

    public TurretSubsystem(Supplier<Pose> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        turretMotor = new TalonFX(TURRET_MOTOR_ID);

        //TODO: Add any sensor to mechanism ratio

        turretConfig.Slot0.kP = kP;
        turretConfig.Slot0.kI = kI;
        turretConfig.Slot0.kD = kD;
        turretConfig.MotionMagic.MotionMagicAcceleration = kACCELERATION;
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = kCRUISE_VELOCITY;

        turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        turretMotor.getConfigurator().apply(turretConfig);
    }

    /**
     * The turret's default command constantly tracks the goal based on our pose supplier
     */
    public Command turretDefaultCommand() {
        return run(() -> turretMotor.setControl(turretController.withPosition(MathUtil.clamp(getTurretTargetRotations(), MIN_TURRET_ROTATIONS, MAX_TURRET_ROTATIONS))));
    }

    private double getTurretTargetRotations() {
        if (hubPose == null) 
            hubPose = (DriverStation.getAlliance().get() == Alliance.Blue) ? 
                       FieldConstants.BLUE_HUB_POSE :
                       FieldConstants.RED_HUB_POSE;
        
        Pose robotPose = robotPoseSupplier.get();

        double turretXGlobal = robotPose.x + TURRET_ORIGIN_OFFSET_X_INCHES * Math.cos(robotPose.heading);
        double turretYGlobal = robotPose.y + TURRET_ORIGIN_OFFSET_Y_INCHES * Math.sin(robotPose.heading);

        double targetAngleRadians = Math.atan2(hubPose.y - turretYGlobal, hubPose.x - turretXGlobal);

        return Units.radiansToRotations(robotPose.heading - targetAngleRadians);
    }

    private double getTurretRotations() {
        return turretMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        DogLog.forceNt.log("Turret/turret_angle_degrees", Units.rotationsToDegrees(getTurretRotations()));
    }
}
