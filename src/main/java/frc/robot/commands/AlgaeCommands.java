// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import java.util.function.Supplier;

/** Add your docs here. */
public class AlgaeCommands {
    public static Command whackAlgae(Drive drive, SuperStructure superStructure, Intake intake, boolean L3Algae) {
        Supplier<Pose2d> poseSupplier = () -> {
            Pose2d pose = drive.getPose().nearest(FieldConstants.Reef.getAllianceReefList());

            return pose.transformBy(new Transform2d(0.48, 0, Rotation2d.fromDegrees(180)));
        };

        return DriveCommands.driveToPose(drive, poseSupplier)
                .andThen(superStructure.moveToPose(L3Algae ? SuperStructurePose.L4 : SuperStructurePose.L3))
                .andThen(superStructure.moveToPose(SuperStructurePose.TROUGH));
    }
}
