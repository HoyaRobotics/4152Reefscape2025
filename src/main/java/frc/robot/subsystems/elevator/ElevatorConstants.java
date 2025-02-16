// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class ElevatorConstants {
    public static final double positionErrorInches = 0.05;

    public class l_Positions {
        // TODO: Add actual values
        public static final Distance Base = Inches.of(0.0),
                Trough = Inches.of(0),
                L2 = Inches.of(20),
                L3 = Inches.of(30),
                L4 = Inches.of(55),
                Loading = Inches.of(15.0),
                Processing = Inches.of(0.0);
    }
}
