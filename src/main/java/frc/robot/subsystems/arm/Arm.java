// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private final ArmIO io;
    private ArmInputsAutoLogged inputs;

    /** Creates a new Arm. */
    public Arm(ArmIO io) {
        this.io = io;
        this.inputs = new ArmInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    void setIntakeSpeed(double speed) {}

    void setArmPosition(Angle targetAngle) {
        this.io.setArmPosition(targetAngle);
    }

    boolean hasGamePiece() {
        return this.inputs.hasGamePiece;
    }

    boolean isArmAtPosition(Angle queriedAngle) {
        // TODO: implement
        return false;
    }

    Angle getArmPosition() {
        return this.inputs.armAngle;
    }

    void stopIntake() {}

    void stopArm() {}
}
