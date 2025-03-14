// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeIntakeAction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeAction;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import java.util.List;
import java.util.Set;

/** Add your docs here. */
public class AlgaeCommands {
    public static Command removeL2AlgaeV1(SuperStructure superStructure, Intake intake) {
        return superStructure
                .moveToPose(SuperStructurePose.L2_ALGAE)
                .andThen(superStructure
                        .moveToPose(SuperStructurePose.L3_ALGAE)
                        .deadlineFor(intake.run(IntakeAction.PLACING)));
    }

    public static Command removeAlgaeV2(SuperStructure superStructure, AlgaeIntake algaeIntake, Drive drive) {

        return new DeferredCommand(
                () -> {
                    List<SuperStructurePose> algaePoses =
                            SuperStructure.getAlgaePoses(Reef.getNearestAlgaePoses(drive));

                    // is this correct?
                    return superStructure
                            .moveToPose(algaePoses.get(0))
                            .andThen(superStructure.moveToPose(algaePoses.get(1)))
                            .andThen(superStructure.moveToPose(algaePoses.get(2)))
                            .deadlineFor(algaeIntake.run(AlgaeIntakeAction.INTAKING));
                },
                Set.of(superStructure.arm, superStructure.elevator));
    }

    public static Command preStageRemoveAlgaeV2(SuperStructure superStructure, AlgaeIntake algaeIntake, Drive drive) {

        return new DeferredCommand(
                () -> {
                    List<SuperStructurePose> algaePoses =
                            SuperStructure.getAlgaePoses(Reef.getNearestAlgaePoses(drive));

                    // is this correct?
                    return superStructure
                            .moveToPose(algaePoses.get(0));
                },
                Set.of(superStructure.arm, superStructure.elevator));
    }


    public static Command scoreAlgaeInNet(SuperStructure superStructure, AlgaeIntake algaeIntake) {
        return superStructure
                .moveToPose(SuperStructurePose.ALGAE_NET)
                .andThen(algaeIntake.run(AlgaeIntakeAction.NET).withTimeout(AlgaeIntakeConstants.PlacingTimeout));
    }
}
