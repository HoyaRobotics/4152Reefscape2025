// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private IntakeInputsAutoLogged inputs;
    /** Creates a new Intake. */
    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        this.io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void setSpeed(AngularVelocity targetSpeed) {
        this.io.setSpeed(targetSpeed);
    }

    public void stopIntake() {
        this.io.stop();
    }
}
