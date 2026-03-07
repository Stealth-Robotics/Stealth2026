package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private DisplayMode currentDisplayMode = DisplayMode.DISABLED;

    private final CANdle candle;
    private final CANdleConfiguration candleConfig;

    private final SolidColor hubActiveAnimation = new SolidColor(0, 7)
        .withColor(new RGBWColor(12, 199, 93));

    private final SolidColor hubInactiveAnimation = new SolidColor(0, 7)
        .withColor(new RGBWColor(199, 34, 12));

    private final ColorFlowAnimation disabledAnimation = new ColorFlowAnimation(0, 7)
        .withFrameRate(10)
        .withColor(new RGBWColor(0, 96, 148));

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
        
        candleConfig.LED.BrightnessScalar = 0.1;

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
