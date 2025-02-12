// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {
    @AutoLog
    class ArmInputs {
        public Angle armAngle;
    }

    default void updateInputs(ArmInputs inputs) {}

    default void setPosition(Angle targetAngle) {}

    default void stop() {}
}
