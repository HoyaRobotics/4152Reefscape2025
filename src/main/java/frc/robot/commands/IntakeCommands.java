// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

/** Add your docs here. */
public class IntakeCommands {
    private IntakeCommands() {}

    public static Command RunIntake(Intake intake, AngularVelocity speed, Current currentLimit) {
        return Commands.run(() -> {}, intake).beforeStarting(() -> intake.setSpeed(speed));
        // .finallyDo(() -> intake.stopIntake());
    }

    public static Command RunIntakeTimeout(
            Intake intake, AngularVelocity speed, Current currentLimit, double timeoutSeconds) {
        return RunIntake(intake, speed, currentLimit).withTimeout(timeoutSeconds);
    }

    public static Command RunIntakeTillSensed(Intake intake, AngularVelocity speed, Current currentLimit) {
        return RunIntake(intake, speed, currentLimit).until(() -> intake.hasCoral());
    }

    public static Command RunIntakeTillEmpty(Intake intake, AngularVelocity speed, Current currentLimit) {
        return RunIntake(intake, speed, currentLimit).until(() -> !intake.hasCoral());
    }

    public static Command StopIntake(Intake intake) {
        return Commands.runOnce(() -> intake.stopIntake(), intake);
    }

    public static Command HoldIntake(Intake intake) {
        return Commands.runOnce(() -> intake.setSpeed(IntakeConstants.IntakeSpeeds.holding), intake);
    }
}
