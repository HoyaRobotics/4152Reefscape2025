// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

    private final ClimberIO io;
    private ClimberInputsAutoLogged inputs;
    /** Creates a new Climber. */
    public Climber(ClimberIO io) {
        this.io = io;
        this.inputs = new ClimberInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        this.io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public void setVoltage(Voltage volts) {
        io.setVoltage(volts);
    }

    public void setAngle(Angle targetAngle, boolean fast) {
        io.setAngle(targetAngle, fast);
    }

    boolean isAtPosition() {
        return false;
        // TODO: implement
    }
}
