// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.arm;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ArmIOSim implements ArmIO {

    private SingleJointedArmSim armSim;
    private PIDController armController = new PIDController(100.0, 0, 0);

    public ArmIOSim() {
        armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                185.7143,
                0.16931712,
                0.28778209,
                ArmConstants.baseAngle.in(Radians),
                Units.degreesToRadians(180.0),
                false,
                ArmConstants.startingAngle.in(Radians));
        armController.setTolerance(2.0);
        setPosition(ArmConstants.startingAngle);
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        armSim.setInputVoltage(armController.calculate(armSim.getAngleRads()));
        armSim.update(0.02);
        inputs.armAngle = Radians.of(armSim.getAngleRads());
    }

    @Override
    public void setPosition(Angle targetAngle) {
        armController.setSetpoint(targetAngle.in(Radians));
    }

    @Override
    public void stop() {}
}
