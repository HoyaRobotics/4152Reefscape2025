// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        Pose3d climberPose = new Pose3d(
                Inches.of(0),
                Inches.of(12),
                Inches.of(14.875),
                new Rotation3d(inputs.climberAngle.unaryMinus(), Degrees.of(0), Degrees.of(0)));
        Logger.recordOutput("Climber/ClimberPose", climberPose);
    }

    public void setVoltage(Voltage volts) {
        io.setVoltage(volts);
    }

    public void setAngle(Angle targetAngle, boolean fast) {
        io.setAngle(targetAngle, fast);
    }

    public Command runVoltage(Voltage targetVoltage) {
        return Commands.run(() -> setVoltage(targetVoltage), this);
    }

    public Command runAngle(Angle targetAngle, boolean fast) {
        return Commands.runOnce(() -> setAngle(targetAngle, fast), this);
    }

    boolean isAtPosition() {
        return false;
        // TODO: implement
    }
}
