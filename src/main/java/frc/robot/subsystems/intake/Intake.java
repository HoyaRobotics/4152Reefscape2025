// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    public class IntakeAction {
        public final Current currentLimit;
        public final AngularVelocity speed;

        public IntakeAction(AngularVelocity speed, Current currentLimit) {
            this.speed = speed;
            this.currentLimit = currentLimit;
        }
    }

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

    // make pose a supplier?
    private IntakeAction actionFromPose(SuperStructurePose pose, boolean intaking) {
        Current currentLimit = pose == SuperStructurePose.L4 ? Amps.of(40) : Amps.of(20);
        AngularVelocity speed;
        if (intaking) {
            speed = IntakeConstants.IntakingSpeed;
        } else {
            speed = pose == SuperStructurePose.TROUGH ? IntakeConstants.TroughSpeed : IntakeConstants.PlacingSpeed;
        }
        return new IntakeAction(speed, currentLimit);
    }

    // what commands do we need?
    // intake, outtake, sensed versions, for timeout we can just add decorator.
    public Command run(SuperStructurePose pose, boolean intaking) {
        return Commands.run(() -> {}, this).beforeStarting(() -> {
            IntakeAction action = actionFromPose(pose, intaking);
            setSpeed(action.speed);
            setCurrentLimit(action.currentLimit);
        });
    }

    public Command runRaw(AngularVelocity speed, Current currentLimit) {
        return Commands.run(() -> {}, this).beforeStarting(() -> {
            setSpeed(speed);
            setCurrentLimit(currentLimit);
        });
    }

    public Command runWithSensor(SuperStructurePose pose, boolean intaking) {
        return run(pose, intaking).until(() -> {
            return intaking ? this.hasCoral() : !this.hasCoral();
        });
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
