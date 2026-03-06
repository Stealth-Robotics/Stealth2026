package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private DisplayMode currentDisplayMode = DisplayMode.DISABLED;

    private final CANdle candle;
    private final CANdleConfiguration candleConfig;

    private final SolidColor hubActiveAnimation = new SolidColor(0, 0)
        .withColor(new RGBWColor(12, 199, 93));

    private final SolidColor hubInactiveAnimation = new SolidColor(0, 0)
        .withColor(new RGBWColor(199, 34, 12));

    private final LarsonAnimation disabledAnimation = new LarsonAnimation(0, 0)
        .withColor(new RGBWColor(127, 12, 199))
        .withBounceMode(LarsonBounceValue.Front)
        .withFrameRate(25);

    private boolean applyAnimation = true;

    public enum DisplayMode {
        DISABLED,
        HUB_ACTIVE,
        HUB_INACTIVE
    }

    //TODO: Find CAN ID
    private final int CANDLE_ID = 0;

    public LEDSubsystem() {
        candle = new CANdle(CANDLE_ID);
        candleConfig = new CANdleConfiguration();

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
