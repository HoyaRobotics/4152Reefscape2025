// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorInputs {
        public Distance position;
    }

    default void setPosition(Distance targetPosition, boolean motionMagic) {}

    default void setVoltage(Voltage voltage) {}

    default Current getCurrent() {
        return Amps.of(0.0);
    }

    default void stop() {}

    default LinearVelocity getVelocity() {
        return MetersPerSecond.of(0);
    }

    default void updateInputs(ElevatorInputs inputs) {}

    default void zeroEncoder() {}

    default void changeSoftLimits(boolean enable) {}
}
