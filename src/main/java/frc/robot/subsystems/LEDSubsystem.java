package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDSubsystem extends SubsystemBase {
    private final BooleanSupplier hubActive;

    private boolean isDisabled = false;

    private final CANdle candle;
    private final CANdleConfiguration candleConfig;

    private final RGBWColor greenColor = new RGBWColor(0, 255, 0);
    private final RGBWColor redColor = new RGBWColor(255, 0, 0);

    private final SolidColor hubActiveAnimation = new SolidColor(8, 18)
        .withColor(greenColor);

    private final SolidColor hubInactiveAnimation = new SolidColor(8, 18)
        .withColor(redColor);

    private final StrobeAnimation blinkAnimation = new StrobeAnimation(8, 18)
        .withFrameRate(10);

    private final SingleFadeAnimation disabledAnimation = new SingleFadeAnimation(8, 18)
        .withColor(redColor)
        .withFrameRate(50);

    private boolean blinking = false;

    public enum DisplayMode {
        DISABLED,
        HUB_ACTIVE,
        HUB_INACTIVE
    }

    private final int CANDLE_ID = 34;

    public LEDSubsystem(BooleanSupplier hubActive) {
        this.hubActive = hubActive;

        candle = new CANdle(CANDLE_ID);
        candleConfig = new CANdleConfiguration();
        
        candleConfig.LED.BrightnessScalar = 0.05;
        candleConfig.LED.StripType = StripTypeValue.BRG;
        candleConfig.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;

        candle.getConfigurator().apply(candleConfig);
    }

    public void setLEDBrightness(double value) {
        candle.getConfigurator().apply(candleConfig.LED.withBrightnessScalar(value));
    }

    public void setIsDisabled(boolean value) {
        isDisabled = value;
        candle.clearAllAnimations();
    }

    public boolean isBlinking() {
        return blinking;
    }

    public Command blink() {
        Command blinkCommand = new SequentialCommandGroup(
            new InstantCommand(() -> {
                candle.clearAllAnimations();
                candle.setControl(blinkAnimation.withColor(
                    hubActive.getAsBoolean() ? greenColor : redColor
                ));
            }, this),
            new WaitCommand(6) //Blink for this many seconds
        ).beforeStarting(() -> blinking = true)
        .finallyDo(() -> blinking = false);

        blinkCommand.addRequirements(this);

        return blinkCommand;
    }

    @Override
    public void periodic() {
        if (isDisabled)
            candle.setControl(disabledAnimation);
        else if (!blinking) {
            boolean isHubActive = hubActive.getAsBoolean();

            candle.clearAllAnimations();

            if (isHubActive)
                candle.setControl(hubActiveAnimation);
            else candle.setControl(hubInactiveAnimation);
        }
    }
}
