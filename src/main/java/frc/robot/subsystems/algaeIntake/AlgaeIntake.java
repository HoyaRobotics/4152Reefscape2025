// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeIntakeAction;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {

    private final AlgaeIntakeIO io;
    private AlgaeIntakeInputsAutoLogged inputs;

    private Current currentLimit = Amps.of(0);

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

    public Command run(AlgaeIntakeAction intaking) {
        return Commands.run(() -> {}, this).beforeStarting(() -> {
            io.setCurrentLimit(intaking.currentLimit);
            io.setSpeed(intaking.speed);
        });
    }

    public void runIntake(AlgaeIntakeAction intaking) {
        if (intaking.currentLimit != currentLimit) {
            io.setCurrentLimit(intaking.currentLimit);
            currentLimit = intaking.currentLimit;
        }
        io.setSpeed(intaking.speed);
    }

    public Command runWithSensor(AlgaeIntakeAction algaeIntakeAction) {
        return run(algaeIntakeAction).until(() -> {
            return algaeIntakeAction.speed.in(RevolutionsPerSecond) >= 0 ? this.hasAlgae() : !this.hasAlgae();
        });
    }

    public void stopIntake() {
        this.io.stop();
    }

    public boolean hasAlgae() {
        return inputs.hasAlgae;
    }
}
