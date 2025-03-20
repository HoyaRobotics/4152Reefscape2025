// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

/** Add your docs here. */
public class LEDIOReal implements LEDIO {
    private final CANdle candle = new CANdle(3);

    public LEDIOReal() {
        configureCANdle();
    }

    private void configureCANdle() {
        candle.configFactoryDefault();
        candle.configStatusLedState(false);
        candle.configLOSBehavior(false);
        candle.configV5Enabled(true);
        candle.configVBatOutput(VBatOutputMode.Off);
        candle.configBrightnessScalar(1.0);
        candle.configLEDType(LEDStripType.GRB);
    }

    @Override
    public void setLED(int r, int g, int b, int start, int end) {
        int count = end - start + 1;
        candle.setLEDs(r, g, b, 0, start, count);
    }

    @Override
    public void updateInputs(LEDInputs inputs) {}
}
