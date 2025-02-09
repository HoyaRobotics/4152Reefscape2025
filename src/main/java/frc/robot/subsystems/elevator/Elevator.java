// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//McT testing Git
package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private ElevatorInputsAutoLogged inputs;

    /** Creates a new Elevator. */
    public Elevator(ElevatorIO io) {
        this.io = io;
        this.inputs = new ElevatorInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    void stop() {
        this.io.stop();
    }

    boolean isAtPosition(double queriedPosition) {
        double error = Math.abs(queriedPosition - this.inputs.position);

        return error < ElevatorConstants.positionError;
    }

    double getPosition() {
        return this.inputs.position;
    }

    void setPosition(double targetPosition) {
        this.io.setPosition(targetPosition);
    }
}
