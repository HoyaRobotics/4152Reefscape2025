// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

/** Add your docs here. */
public class IntakeConstants {

    public class IntakeAction {
        public final Current currentLimit;
        public final AngularVelocity speed;

        public IntakeAction(AngularVelocity speed, Current currentLimit) {
            this.speed = speed;
            this.currentLimit = currentLimit;
        }
    }

    public static final AngularVelocity IntakingSpeed = RevolutionsPerSecond.of(15.0);
    public static final AngularVelocity TroughSpeed = RevolutionsPerSecond.of(-8.0);
    public static final AngularVelocity PlacingSpeed = RevolutionsPerSecond.of(-20.0);
    public static final AngularVelocity HoldingSpeed = RevolutionsPerSecond.of(1.0);

    // 23 RPS max intake speed
    public static class IntakeSpeeds {
        public static final AngularVelocity intaking = RevolutionsPerSecond.of(15.0),
                placing = RevolutionsPerSecond.of(-20.0),
                placingTrough = RevolutionsPerSecond.of(-8.0),
                holding = RevolutionsPerSecond.of(1.0),
                empty = RevolutionsPerSecond.of(0.0);
    }

    public static class CurrentLimits {
        public static final Current L4 = Amps.of(40),
                everything = Amps.of(20),
                receiving = Amps.of(20),
                holding = Amps.of(20);
    }

    public static final double PlacingTimeout = 0.35;
    public static final double PostPlacingTimeout = 0.35;
}
