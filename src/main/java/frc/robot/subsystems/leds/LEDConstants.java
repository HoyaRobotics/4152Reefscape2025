// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

/** Add your docs here. */
public class LEDConstants {
    public enum LEDColors {
        EMPTY(255, 0, 0),
        FLASHING(255, 255, 255),
        OFF(0, 0, 0),
        HOLDING(0, 255, 0);
        public final int r, g, b;

        LEDColors(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    public static double flashingOnTimeout = 0.15;
    public static double flashingOffTimeout = 0.15;
}
