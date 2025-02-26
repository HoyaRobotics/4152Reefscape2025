// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ClimberIOSim implements ClimberIO {

    private SingleJointedArmSim climberSim;
    private PIDController climberController = new PIDController(100.0, 0, 0);

    public ClimberIOSim() {
        climberSim = new SingleJointedArmSim(
                DCMotor.getNeoVortex(1),
                251.8519,
                0.16931712,
                0.28778209,
                Rotations.of(-0.26).in(Radians),
                Rotations.of(0.3).in(Radians),
                false,
                Rotations.of(-0.25).in(Radians));
        climberController.setTolerance(2.0);
        setAngle(Rotations.of(-0.25), true);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        climberSim.setInputVoltage(climberController.calculate(climberSim.getAngleRads()));
        climberSim.update(0.02);
        inputs.climberAngle = Radians.of(climberSim.getAngleRads());
    }

    @Override
    public void setAngle(Angle targetAngle, boolean fast) {
        climberController.setSetpoint(targetAngle.in(Radians));
    }
}
