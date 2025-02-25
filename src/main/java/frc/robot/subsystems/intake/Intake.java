// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private IntakeInputsAutoLogged inputs;
    private Elevator elevator;
    private Arm arm;
    /** Creates a new Intake. */
    public Intake(IntakeIO io, Elevator elevator, Arm arm) {
        this.io = io;
        this.elevator = elevator;
        this.arm = arm;
        inputs = new IntakeInputsAutoLogged();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        this.io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        // Pose3d coralPose = new Pose3d(0, 0, 0, new Rotation3d(Degrees.of(0), arm.getArmPosition(), Degrees.of(0)));
        // Logger.recordOutput("IntakeCoral", coralPose);
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

    public boolean hasCoral() {
        return inputs.hasCoral;
    }
}
