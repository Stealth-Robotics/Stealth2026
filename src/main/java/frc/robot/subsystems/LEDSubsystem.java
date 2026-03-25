package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private DisplayMode currentDisplayMode = DisplayMode.DISABLED;

    private final CANdle candle;
    private final CANdleConfiguration candleConfig;

    private final SolidColor hubActiveAnimation = new SolidColor(8, 18)
        .withColor(new RGBWColor(12, 199, 93));

    private final SolidColor hubInactiveAnimation = new SolidColor(8, 18)
        .withColor(new RGBWColor(199, 34, 12));

    private final RainbowAnimation disabledAnimation = new RainbowAnimation(8, 18)
        .withFrameRate(20);

    private boolean applyAnimation = true;

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
        applyAnimation = true;
    }

    @Override
    public void periodic() {
        if (applyAnimation) {
            candle.setControl(
                currentDisplayMode.equals(DisplayMode.HUB_ACTIVE) ? 
                    hubActiveAnimation : currentDisplayMode.equals(DisplayMode.HUB_INACTIVE) ?
                    hubInactiveAnimation : disabledAnimation
            );
            applyAnimation = false;
        }
    }
}
