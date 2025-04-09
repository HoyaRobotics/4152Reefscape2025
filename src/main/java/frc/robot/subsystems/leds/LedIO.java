package frc.robot.subsystems.leds;

import frc.robot.subsystems.leds.LED.LEDState;

public interface LedIO {
    default void requestState(LEDState nextState) {}
}
