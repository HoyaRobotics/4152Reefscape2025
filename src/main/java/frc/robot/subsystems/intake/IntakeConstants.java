// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class IntakeConstants {
    public class IntakeSpeeds {
        public static final AngularVelocity intaking = RevolutionsPerSecond.of(15.0),
                placing = RevolutionsPerSecond.of(-6.0);
    }

    public static final double PlacingTimeout = 0.5;
}
