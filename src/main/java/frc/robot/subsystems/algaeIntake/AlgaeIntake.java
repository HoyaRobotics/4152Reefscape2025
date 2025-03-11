// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {

    public class IntakeAction {
        public final Current currentLimit;
        public final AngularVelocity speed;

        public IntakeAction(AngularVelocity speed, Current currentLimit) {
            this.speed = speed;
            this.currentLimit = currentLimit;
        }
    }

    private final AlgaeIntakeIO io;
    private AlgaeIntakeInputsAutoLogged inputs;

    public AlgaeIntake(AlgaeIntakeIO io) {
        this.io = io;
        inputs = new AlgaeIntakeInputsAutoLogged();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("AlgaeIntake", inputs);
        // This method will be called once per scheduler run
    }

    public Command run(AngularVelocity velocity) {
        return Commands.run(() -> {}, this).beforeStarting(() -> {
            setSpeed(velocity);
        });
    }

    public void setSpeed(AngularVelocity targetSpeed) {
        this.io.setSpeed(targetSpeed);
    }

    public void setCurrentLimit(Current currentLimit) {
        this.io.setCurrentLimit(currentLimit);
    }

    public void setVoltage(Voltage voltage) {
        this.io.setVoltage(voltage);
    }

    public void stopIntake() {
        this.io.stop();
    }

    public boolean hasAlgae() {
        return inputs.hasAlgae;
    }
}
