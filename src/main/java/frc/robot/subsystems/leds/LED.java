// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LEDConstants.LEDColors;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
    private final LEDIO io;
    private final LEDInputsAutoLogged inputs;

    public enum LEDState {
        HOLDING_CORAL,
        ALIGNING,
        AUTO,
        PLACING,
        EMPTY
    }

    /** Creates a new LED. */
    public LED(LEDIO io) {
        this.io = io;
        this.inputs = new LEDInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        this.io.updateInputs(inputs);
        Logger.processInputs("LED", inputs);
    }

    public void setLEDs(LEDColors colors) {
        io.setLED(colors.r, colors.g, colors.b, 8, 64);
    }

    public Command LEDCommand(LEDColors colors) {
        return Commands.runOnce(() -> setLEDs(colors), this);
    }
}
