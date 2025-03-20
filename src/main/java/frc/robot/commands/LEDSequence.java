// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.leds.LEDConstants;
import frc.robot.subsystems.leds.LEDConstants.LEDColors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LEDSequence extends SequentialCommandGroup {
    /** Creates a new LEDSequence. */
    public LEDSequence(LED led, Intake intake) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                led.LEDCommand(LEDColors.EMPTY),
                Commands.waitUntil(() -> intake.hasCoral()),
                led.LEDCommand(LEDColors.FLASHING),
                new WaitCommand(LEDConstants.flashingOnTimeout),
                led.LEDCommand(LEDColors.OFF),
                new WaitCommand(LEDConstants.flashingOffTimeout),
                led.LEDCommand(LEDColors.FLASHING),
                new WaitCommand(LEDConstants.flashingOnTimeout),
                led.LEDCommand(LEDColors.OFF),
                new WaitCommand(LEDConstants.flashingOffTimeout),
                led.LEDCommand(LEDColors.FLASHING),
                new WaitCommand(LEDConstants.flashingOnTimeout),
                led.LEDCommand(LEDColors.OFF),
                new WaitCommand(LEDConstants.flashingOffTimeout),
                led.LEDCommand(LEDColors.FLASHING),
                new WaitCommand(LEDConstants.flashingOnTimeout),
                led.LEDCommand(LEDColors.OFF),
                new WaitCommand(LEDConstants.flashingOffTimeout),
                led.LEDCommand(LEDColors.HOLDING),
                Commands.waitUntil(() -> !intake.hasCoral()),
                led.LEDCommand(LEDColors.EMPTY));
    }
}
