package frc.robot.commands;

/*
import java.util.function.Supplier;

import com.google.flatbuffers.FlexBuffers.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef.Side;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;



public class AutoAlign {

    private Command SelectPoseCommand(SuperStructure superStructure) {
        return new SelectCommand<>(
        Map.ofEntries(
                Map.entry(SuperStructurePose.BASE, superStructure.moveToPosePreAngle(SuperStructurePose.L2)),
                Map.entry
        )
    }

    public static Command autoAlignAndPlace(
        CommandXboxController driverController,
        Drive drive,
        SuperStructure superStructure,
        Intake intake,
        Side side
    ){
        Supplier<Pose2d> drivePose = () -> FieldConstants.Reef.offsetReefPose(
                drive.getPose().nearest(FieldConstants.Reef.getAllianceReefList()), side);
        return DriveCommands.driveToPose(drive, drivePose)
            .alongWith()
        return Commands.runOnce(() -> {});
    }
}
*/
