// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class AlgaeIntakeIOSim implements AlgaeIntakeIO {

    public AlgaeIntakeIOSim() {}

    public void addSimulatedGamePiece() {}

    @Override
    public void setSpeed(AngularVelocity targetSpeed) {}

    @Override
    public void stop() {}

    @Override
    public void updateInputs(AlgaeIntakeInputs inputs) {
        inputs.hasAlgae = false;
    }
}
