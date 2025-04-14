// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {
    @AutoLog
    class ArmInputs {
        public Angle armAngle;
    }

    default void updateInputs(ArmInputs inputs) {}

    default AngularVelocity getRotationalVelocity() { return RotationsPerSecond.of(0); }

    default void setPosition(Angle targetAngle, boolean motionMagic) {}

    default void stop() {}
}
