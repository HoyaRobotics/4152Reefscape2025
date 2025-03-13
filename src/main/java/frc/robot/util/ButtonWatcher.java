// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.DriveMap;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;

/** Add your docs here. */
public class ButtonWatcher {
    private final DriveMap controller;
    public SuperStructurePose selectedPose = SuperStructurePose.BASE;

    public ButtonWatcher(DriveMap controller) {
        this.controller = controller;
    }

    public Command WaitSelectPose() {
        return new WaitUntilCommand(() -> {
            if (controller.moveToL2(true).getAsBoolean()) {
                selectedPose = SuperStructurePose.L2;
            } else if (controller.moveToL3(true).getAsBoolean()) {
                selectedPose = SuperStructurePose.L3;
            } else if (controller.moveToL4(true).getAsBoolean()) {
                selectedPose = SuperStructurePose.L4;
            } else {
                return false;
            }
            return true;
        });
    }

    public SuperStructurePose getSelectedPose() {
        return selectedPose;
    }
}
