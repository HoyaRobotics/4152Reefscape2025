// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.elevator;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.*;

/** Add your docs here. */
public class ElevatorConstants {
    public static final double positionErrorInches = 0.05;

    public class l_Positions {
        // TODO: Add actual values
        public static final Distance Base = Inches.of(0.0),
                Trough = Inches.of(0),
                L2 = Inches.of(0),
                L3 = Inches.of(20.875),
                L4 = Inches.of(53.25),
                Loading = Inches.of(0.0),
                Processing = Inches.of(0.0);
    }
}
