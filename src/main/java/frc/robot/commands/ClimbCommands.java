// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;

/** Add your docs here. */
public class ClimbCommands {
    private ClimbCommands() {}

    public static Command moveClimber(Climber climber, Voltage targetVoltage) {
        return Commands.run(() -> climber.setVoltage(targetVoltage), climber);
    }
}
