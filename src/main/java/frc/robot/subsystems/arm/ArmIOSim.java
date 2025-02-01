// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

/** Add your docs here. */
public class ArmIOSim implements ArmIO {

    @Override
    public void updateInputs(ArmInputs inputs) {}

    @Override
    public void setArmPosition(Angle targetAngle) {}

    @Override
    public void setIntakeSPeed(double speed) {}

    @Override
    public void stopIntake() {}

    @Override
    public void stopArm() {}
}
