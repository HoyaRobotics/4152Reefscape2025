// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.Consumer;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {
    private final IntakeSimulation intakeSimulation;
    private final SwerveDriveSimulation driveSimulation;
    private final Consumer<AngularVelocity> placeCoral;

    public IntakeIOSim(SwerveDriveSimulation driveSimulation, Consumer<AngularVelocity> placeCoral) {
        this.driveSimulation = driveSimulation;
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                "Coral", driveSimulation, Inches.of(20), Inches.of(8), IntakeSimulation.IntakeSide.FRONT, 1);
        this.intakeSimulation.addGamePieceToIntake();
        this.placeCoral = placeCoral;
    }

    @Override
    public void setSpeed(AngularVelocity targetSpeed) {
        if (targetSpeed == IntakeConstants.IntakeSpeeds.intaking) {
            intakeSimulation.startIntake();
        } else if (targetSpeed == IntakeConstants.IntakeSpeeds.placing
                || targetSpeed == IntakeConstants.IntakeSpeeds.placingTrough
                        && intakeSimulation.obtainGamePieceFromIntake()) {
            placeCoral.accept(targetSpeed);
        }
    }

    @Override
    public void stop() {
        intakeSimulation.stopIntake();
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.hasCoral = intakeSimulation.getGamePiecesAmount() != 0;
        Logger.recordOutput("Intake/intakePieces", intakeSimulation.getGamePiecesAmount());
    }
}
