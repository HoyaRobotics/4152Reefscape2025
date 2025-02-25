// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class ElevatorConstants {
    public static final Distance positionError = Inches.of(0.5);

    public class l_Positions {
        // TODO: Add actual values
        public static final Distance Base = Inches.of(0.5),
                Trough = Inches.of(9.0), // 0.0
                L2 = Inches.of(3),
                L3 = Inches.of(19),
                L4 = Inches.of(53.0),
                Loading = Inches.of(20.5), // 19.5
                Processing = Inches.of(0.0),
                Barge = Inches.of(52.5),
                Hold = Inches.of(0.0),
                L2Algae = Inches.of(10),
                l3Algae = Inches.of(20);
    }
}
