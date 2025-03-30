// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
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

                    return superStructure
                            .moveToPose(algaePoses.get(0))
                            .andThen(superStructure.moveToPose(algaePoses.get(1)))
                            .withTimeout(0.5)
                            .deadlineFor(algaeIntake.run(AlgaeIntakeAction.INTAKING));
                },
                Set.of(superStructure.arm, superStructure.elevator));
    }

    public static Command preStageRemoveAlgaeV2(SuperStructure superStructure, AlgaeIntake algaeIntake, Drive drive) {

        return Commands.defer(
                () -> {
                    List<SuperStructurePose> algaePoses =
                            SuperStructure.getAlgaePoses(Reef.getNearestAlgaePoses(drive));

                    SuperStructurePose prePose = algaePoses.get(0);
                    return superStructure
                            .moveToPose(prePose)
                            .until(() -> superStructure.arm.withinTolerance(prePose.armAngle, Degrees.of(5))) // 5
                            .deadlineFor(algaeIntake.run(AlgaeIntakeAction.INTAKING));
                },
                Set.of(superStructure.arm, superStructure.elevator));
    }
}
