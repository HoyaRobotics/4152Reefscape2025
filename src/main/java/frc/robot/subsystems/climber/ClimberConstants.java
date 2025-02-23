// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ClimberConstants {

    // TODO: fill in constants

    public static final Angle deployAngle = Degrees.of(90);
    public static final Angle baseAngle = Degrees.of(-90);
    public static final Voltage climbUpVoltage = Volts.of(8.0);
    public static final Voltage climbDownVoltage = Volts.of(-8.0);
}
