// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface LEDIO {
    @AutoLog
    class LEDInputs {
        public double r;
        public double g;
        public double b;
    }

    default void setLEDColor() {}
}
