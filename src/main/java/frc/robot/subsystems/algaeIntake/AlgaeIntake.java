// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeIntakeAction;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {

    private final AlgaeIntakeIO io;
    private AlgaeIntakeInputsAutoLogged inputs;
    private final Elevator elevator;
    private final Arm arm;

    private Current currentLimit = Amps.of(0);

    public AlgaeIntake(AlgaeIntakeIO io, Elevator elevator, Arm arm) {
        this.io = io;
        this.elevator = elevator;
        this.arm = arm;
        inputs = new AlgaeIntakeInputsAutoLogged();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("AlgaeIntake", inputs);
        // This method will be called once per scheduler run
    }

    public Command run(AlgaeIntakeAction intaking) {
        return Commands.run(() -> {}, this).beforeStarting(() -> {
            io.setCurrentLimit(intaking.currentLimit);
            io.setSpeed(intaking.speed);
        });
    }

    public void runIntake(AlgaeIntakeAction intaking) {
        if (intaking.currentLimit != currentLimit) {
            io.setCurrentLimit(intaking.currentLimit);
            currentLimit = intaking.currentLimit;
        }
        io.setSpeed(intaking.speed);
    }

    public Command runWithSensor(AlgaeIntakeAction algaeIntakeAction) {
        return run(algaeIntakeAction).until(() -> {
            return algaeIntakeAction.speed.in(RevolutionsPerSecond) >= 0 ? this.hasAlgae() : !this.hasAlgae();
        });
    }

    public void stopIntake() {
        this.io.stop();
    }

    public boolean hasAlgae() {
        return inputs.hasAlgae;
    }

    public void visualizeAlgaeInIntake(Pose2d robotPose) {
        Pose3d algaePose = new Pose3d();
        // algae position relative to arm pivot point
        algaePose = algaePose.transformBy(new Transform3d(
                Inches.of(24.0), // outwards from elevator
                Inches.of(0),
                Inches.of(0),
                Rotation3d.kZero));

        // roll, pitch, yaw
        algaePose = algaePose.rotateBy(new Rotation3d(
                Degrees.of(0),
                arm.getArmPosition().times(-1).plus(Degrees.of(13)),
                robotPose.getRotation().getMeasure()));
        algaePose = algaePose.plus(new Transform3d(
                robotPose.getMeasureX(),
                robotPose.getMeasureY(),
                Inches.of(21.875).plus(elevator.getPosition()),
                Rotation3d.kZero));

        Logger.recordOutput("AlgaeIntake/AlgaeInIntake", inputs.hasAlgae ? algaePose : Pose3d.kZero);
    }
}
