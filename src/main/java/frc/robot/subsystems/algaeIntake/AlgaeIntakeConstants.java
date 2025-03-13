// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

/** Add your docs here. */
public class AlgaeIntakeConstants {
    public enum AlgaeIntakeAction {
        HOLDING(Amps.of(40), RevolutionsPerSecond.of(15.0)),
        INTAKING(Amps.of(40), RevolutionsPerSecond.of(15.0)),
        PROCESSOR(Amps.of(10), RevolutionsPerSecond.of(-8.0)),
        NET(Amps.of(40), RevolutionsPerSecond.of(-22.0)),
        EMPTY(Amps.of(10), RevolutionsPerSecond.of(2.0));

        public final Current currentLimit;
        public final AngularVelocity speed;

        AlgaeIntakeAction(Current currentLimit, AngularVelocity speed) {
            this.currentLimit = currentLimit;
            this.speed = speed;
        }
    }

    public static final double PlacingTimeout = 0.5; // 0.30
    public static final double PostPlacingTimeout = 0.35;
}
