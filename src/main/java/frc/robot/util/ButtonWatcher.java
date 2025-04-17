// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.DriveMap;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import java.util.Optional;

/** Add your docs here. */
public class ButtonWatcher {
    private final DriveMap controller;
    public Optional<SuperStructurePose> selectedPose = Optional.empty();

    public ButtonWatcher(DriveMap controller) {
        this.controller = controller;

        controller
                .moveToTrough(true)
                .onTrue(Commands.runOnce(() -> selectedPose = Optional.of(SuperStructurePose.TROUGH)));
        controller.moveToL2(true).onTrue(Commands.runOnce(() -> selectedPose = Optional.of(SuperStructurePose.L2)));
        controller.moveToL3(true).onTrue(Commands.runOnce(() -> selectedPose = Optional.of(SuperStructurePose.L3)));
        controller.moveToL4(true).onTrue(Commands.runOnce(() -> selectedPose = Optional.of(SuperStructurePose.L4)));
    }

    public Command WaitSelectPose() {
        return new WaitUntilCommand(() -> selectedPose.isPresent());
    }

    public SuperStructurePose getSelectedPose() {
        return selectedPose.get();
    }
}
