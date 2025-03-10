// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;

/** Add your docs here. */
public class AlgaeCommands {
    public static Command removeL2AlgaeV1(SuperStructure superStructure, Intake intake) {
        return superStructure
                .moveToPose(SuperStructurePose.L2_ALGAE)
                .andThen(superStructure.moveToPose(SuperStructurePose.L3_ALGAE).deadlineFor(intake.run(false)));
    }

    public static Command removeL2AlgaeV2(SuperStructure superStructure, AlgaeIntake algaeIntake) {
        return superStructure
                .moveToPose(SuperStructurePose.L2_ALGAE)
                .andThen(superStructure.moveToPose(SuperStructurePose.L2_ALGAE_GRAB))
                .andThen(superStructure.moveToPose(SuperStructurePose.L2_ALGAE_REMOVE))
                .deadlineFor(algaeIntake.runIntake(true));
    }

    public static Command removeL3AlgaeV2(SuperStructure superStructure, AlgaeIntake algaeIntake) {
        return superStructure
                .moveToPose(SuperStructurePose.L3_ALGAE)
                .andThen(superStructure.moveToPose(SuperStructurePose.L3_ALGAE_GRAB))
                .andThen(superStructure.moveToPose(SuperStructurePose.L3_ALGAE_REMOVE))
                .deadlineFor(algaeIntake.runIntake(true));
    }
}
