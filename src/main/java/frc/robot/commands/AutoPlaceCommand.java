// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.subsystems.superstructure.arm.ArmConstants;

/** Add your docs here. */
public class AutoPlaceCommand extends SequentialCommandGroup {
    public AutoPlaceCommand(SuperStructure superStructure, Intake intake, SuperStructurePose pose) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                superStructure.moveToPose(pose),
                intake.run(false)
                        .withTimeout(IntakeConstants.PlacingTimeout)
                        .andThen(intake.runWithSensor(false)
                                .alongWith(superStructure.retractArm(ArmConstants.baseAngle))
                                .until(superStructure.waitTillRetracted())));
    }
}
