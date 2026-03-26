package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDSubsystem extends SubsystemBase {
    private DisplayMode currentDisplayMode = DisplayMode.DISABLED;

    private final CANdle candle;
    private final CANdleConfiguration candleConfig;

    private final RGBWColor greenColor = new RGBWColor(0, 255, 0);
    private final RGBWColor redColor = new RGBWColor(255, 0, 0);

    private final SolidColor hubActiveAnimation = new SolidColor(8, 18)
        .withColor(greenColor);

    private final SolidColor hubInactiveAnimation = new SolidColor(8, 18)
        .withColor(redColor);

    private final StrobeAnimation blinkAnimation = new StrobeAnimation(8, 18);

    private final RainbowAnimation disabledAnimation = new RainbowAnimation(8, 18);

    private boolean blinking = false;

    public enum DisplayMode {
        DISABLED,
        HUB_ACTIVE,
        HUB_INACTIVE
    }

    private final int CANDLE_ID = 34;

    public LEDSubsystem() {
        candle = new CANdle(CANDLE_ID);
        candleConfig = new CANdleConfiguration();
        
        candleConfig.LED.BrightnessScalar = 0.5;
        candleConfig.LED.StripType = StripTypeValue.RGB;
        candleConfig.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;

        candle.getConfigurator().apply(candleConfig);
    }

    public void changeDisplayMode(DisplayMode mode) {
        currentDisplayMode = mode;
    }

    public Command blink() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                candle.setControl(blinkAnimation.withColor(
                    (currentDisplayMode.equals(DisplayMode.HUB_ACTIVE) ? greenColor : redColor)
                ));
            }),
            new WaitCommand(5) //Blink for this many seconds
        )
        .beforeStarting(() -> blinking = true)
        .finallyDo(() -> blinking = false);
    }

    @Override
    public void periodic() {
        if (!blinking) {
            candle.setControl(
                switch(currentDisplayMode) {
                    case HUB_ACTIVE -> hubActiveAnimation;
                    case HUB_INACTIVE -> hubInactiveAnimation;
                    default -> disabledAnimation;
                }
            );
        }

        DogLog.log("LEDS/display_mode", currentDisplayMode.name());
    }
}
