// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

    private ElevatorSim elevatorSim;
    private PIDController elevatorController = new PIDController(100.0, 0, 0);

    public ElevatorIOSim() {
        elevatorSim = new ElevatorSim(
                DCMotor.getFalcon500(2), 8, 12.63380394, 0.0573 / 2, 0, Units.inchesToMeters(53.25), false, 0.0);
        elevatorController.setTolerance(0.02);
    }

    @Override
    public void setPosition(Distance targetPosition, boolean motionMagic) {
        elevatorController.setSetpoint(targetPosition.in(Meters));
    }

    @Override
    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond());
    }

    @Override
    public void stop() {}

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        elevatorSim.setInputVoltage(elevatorController.calculate(elevatorSim.getPositionMeters()));
        elevatorSim.update(0.02);
        inputs.position = Meters.of(elevatorSim.getPositionMeters());
    }
}
