// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

/** Add your docs here. */
public class ElevatorConstants {
    public static final double positionError = 0.05;

    class ElevatorPosition {
        // TODO: Add actual values
        public static final double Base = 0.0,
                Trough = 0.0,
                Level1 = 0.0,
                Level2 = 0.0,
                Level3 = 0.0,
                Loading = 0.0,
                Processing = 0.0;
    }

    public static ElevatorPosition position;
}
