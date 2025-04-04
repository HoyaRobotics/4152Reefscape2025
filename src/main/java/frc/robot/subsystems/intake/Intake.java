// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.leds.LED.LEDState;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    /*
    public class IntakeAction {
        public final Current currentLimit;
        public final AngularVelocity speed;

        public IntakeAction(AngularVelocity speed, Current currentLimit) {
            this.speed = speed;
            this.currentLimit = currentLimit;
        }
    }
        */

    private final IntakeIO io;
    private Supplier<SuperStructurePose> poseSupplier;
    private IntakeInputsAutoLogged inputs;
    private final Elevator elevator;
    private final Arm arm;
    private final LED leds;

    private Current currentLimit = Amps.of(0);
    /** Creates a new Intake. */
    public Intake(IntakeIO io, Elevator elevator, Arm arm, LED leds) {
        this.io = io;
        this.elevator = elevator;
        this.arm = arm;
        this.leds = leds;
        inputs = new IntakeInputsAutoLogged();
    }

    public void setPoseSupplier(Supplier<SuperStructurePose> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        this.io.updateInputs(inputs);
        if (inputs.hasCoral) {
            leds.requestState(LEDState.HOLDING_CORAL);
        } else {
            leds.requestState(LEDState.EMPTY);
        }
        Logger.processInputs("Intake", inputs);
        // Pose3d coralPose = new Pose3d(0, 0, 0, new Rotation3d(Degrees.of(0), arm.getArmPosition(), Degrees.of(0)));
        // Logger.recordOutput("IntakeCoral", coralPose);
    }

    public void addSimulatedGamePiece() {
        this.io.addSimulatedGamePiece();
    }

    public Command run(IntakeConstants.IntakeAction intakeAction) {
        return Commands.run(() -> {}, this).beforeStarting(() -> {
            io.setCurrentLimit(intakeAction.currentLimit);
            io.setSpeed(intakeAction.speed);
        });
    }

    public void runIntake(IntakeConstants.IntakeAction intakeAction) {
        if (intakeAction.currentLimit != currentLimit) {
            io.setCurrentLimit(intakeAction.currentLimit);
            currentLimit = intakeAction.currentLimit;
        }
        io.setSpeed(intakeAction.speed);
    }

    public Command runWithSensor(IntakeConstants.IntakeAction intakeAction) {
        return run(intakeAction).until(() -> {
            return intakeAction.speed.in(RevolutionsPerSecond) >= 0 ? this.hasCoral() : !this.hasCoral();
        });
    }

    public void stopIntake() {
        this.io.stop();
    }

    public boolean hasCoral() {
        return inputs.hasCoral;
    }
}
