// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeAction;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import java.util.function.BooleanSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class PlacingCommandTrough extends SequentialCommandGroup {
    /** Creates a new PlacingCommand. */
    public PlacingCommandTrough(SuperStructure superStructure, Intake intake, BooleanSupplier placeObject) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                superStructure.moveToPose(SuperStructurePose.TROUGH),
                new WaitUntilCommand(placeObject),
                intake.runWithSensor(IntakeAction.TROUGH)
                        // .andThen(Commands.waitSeconds(IntakeConstants.PostSensingTimeout))
                        .andThen(intake.run(IntakeAction.TROUGH)
                                .withDeadline(superStructure.moveToPose(SuperStructurePose.POST_TROUGH)))
                        .andThen(Commands.waitSeconds(IntakeConstants.PostPlacingTimeout))
                        .andThen(superStructure.moveToPose(SuperStructurePose.POST_POST_TROUGH)));
    }
}
