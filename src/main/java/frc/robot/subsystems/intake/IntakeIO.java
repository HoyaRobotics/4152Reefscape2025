// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        public AngularVelocity speed;
        public boolean hasCoral;
    }

    default void setSpeed(AngularVelocity targetSpeed) {}

    default void setVoltage(Voltage voltage) {}

    default void stop() {}

    default void setCurrentLimit(Current currentLimit) {}

    default void updateInputs(IntakeInputs inputs) {}
}
