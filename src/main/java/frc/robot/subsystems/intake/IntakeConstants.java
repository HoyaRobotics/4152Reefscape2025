// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

/** Add your docs here. */
public class IntakeConstants {
    public enum IntakeAction {
        HOLDING(Amps.of(5), RevolutionsPerSecond.of(1.0)),
        INTAKING(Amps.of(20), RevolutionsPerSecond.of(15.0)),
        TROUGH(Amps.of(30), RevolutionsPerSecond.of(-5.0)),
        PLACING(Amps.of(30), RevolutionsPerSecond.of(-25.0)),
        EMPTY(Amps.of(10), RevolutionsPerSecond.of(0.0));

        public final Current currentLimit;
        public final AngularVelocity speed;

        IntakeAction(Current currentLimit, AngularVelocity speed) {
            this.currentLimit = currentLimit;
            this.speed = speed;
        }
    }

    /*public static final AngularVelocity IntakingSpeed = RevolutionsPerSecond.of(15.0);
    public static final AngularVelocity TroughSpeed = RevolutionsPerSecond.of(-8.0);
    public static final AngularVelocity PlacingSpeed = RevolutionsPerSecond.of(-20.0);
    public static final AngularVelocity HoldingSpeed = RevolutionsPerSecond.of(1.0);*/

    public static final double PlacingTimeout = 0.35; // 0.30
    public static final double PostPlacingTimeout = 0.35;
}
