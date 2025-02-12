// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Velocity;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        public double speed;
        public boolean hasGamePiece;
    }

    default void setSpeed(Velocity targetSpeed) {};

    default void stopIntake() {};

    default void updateInputs(IntakeInputs inputs) {};
}
