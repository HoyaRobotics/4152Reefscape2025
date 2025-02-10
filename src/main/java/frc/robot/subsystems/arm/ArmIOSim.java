// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

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
                0.0361366348,
                14.0,
                Units.degreesToRadians(-25.011685),
                Units.degreesToRadians(64.988315 + 90 + 75),
                false,
                Units.degreesToRadians(64.988315 + 90 + 75));
        armController.setTolerance(2.0);
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        armSim.setInputVoltage(armController.calculate(armSim.getAngleRads()));
        armSim.update(0.02);
        inputs.armAngle = Radians.of(armSim.getAngleRads());
    }

    @Override
    public void setArmPosition(Angle targetAngle) {
        armController.setSetpoint(targetAngle.in(Radians));
    }

    @Override
    public void setIntakeSpeed(double speed) {}

    @Override
    public void stopIntake() {}

    @Override
    public void stopArm() {}
}
