// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {
    @AutoLog
    class ClimberInputs {
        public Angle climberAngle;
    }

    default void updateInputs(ClimberInputs inputs) {}

    default void setAngle(Angle targetAngle, boolean fast) {}

    default void setVoltage(Voltage volts) {}
}
