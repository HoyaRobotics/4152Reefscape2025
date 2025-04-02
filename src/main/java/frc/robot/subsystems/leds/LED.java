// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
    private final CANdle leds = new CANdle(3);

    private LEDState currentState = LEDState.NOTHING;

    public enum LEDState {
        NOTHING,
        HOLDING_CORAL,
        ALIGNING,
        AUTO,
        PLACING,
        EMPTY
    }

    /** Creates a new LED. */
    public LED() {
        leds.configFactoryDefault();
        leds.configStatusLedState(false);
        leds.configLOSBehavior(false);
        leds.configV5Enabled(true);
        leds.configVBatOutput(VBatOutputMode.Off);
        leds.configBrightnessScalar(1.0);
        leds.configLEDType(LEDStripType.GRB);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("LED/currentState", currentState.toString());
    }

    // make commands to do things like flashing transition
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
            case PLACING -> leds.animate(new SingleFadeAnimation(255, 0, 255, 0, 0.75, 64));
            default -> {}
        }
        currentState = nextState;
    }
}
