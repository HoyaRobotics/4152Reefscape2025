package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import frc.robot.subsystems.leds.LED.LEDState;

public class LedIOReal implements LedIO {
    private final CANdle leds = new CANdle(3);
    private LEDState currentState = LEDState.NOTHING;

    public LedIOReal() {
        leds.configFactoryDefault();
        leds.configStatusLedState(false);
        leds.configLOSBehavior(false);
        leds.configV5Enabled(true);
        leds.configVBatOutput(VBatOutputMode.Off);
        leds.configBrightnessScalar(1.0);
        leds.configLEDType(LEDStripType.GRB);
    }

    @Override
    public void requestState(LEDState nextState) {
        // define state priority ex. only show coral sensor status
        // if nothing else happening
        if (currentState == nextState) return;

        if ((nextState == LEDState.EMPTY || nextState == LEDState.HOLDING_CORAL)
                && (currentState != LEDState.EMPTY
                        && currentState != LEDState.HOLDING_CORAL
                        && currentState != LEDState.NOTHING)) return;

        leds.clearAnimation(0);
        leds.configBrightnessScalar(1);

        switch (nextState) {
            case EMPTY -> leds.setLEDs(255, 0, 0);
            case HOLDING_CORAL -> leds.setLEDs(0, 255, 0);
            case ALIGNING -> leds.animate(new RainbowAnimation(1.0, 0.5, 64));
            case PLACING -> leds.animate(new SingleFadeAnimation(0, 0, 255, 0, 1.0, 64));
            case DISABLED -> leds.animate(
                    new LarsonAnimation(0, 0, 255, 0, 0.5, 64 - 8, LarsonAnimation.BounceMode.Center, 8, 8));
            default -> {}
        }
        currentState = nextState;
    }
}
