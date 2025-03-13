// Auto drive to closest branch and move to pose indicated by buttons
// on remote, outtaking onto it
// if one of the level buttons is pressed before its ready it will take
// that into account and place with no delay

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeIntakeAction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeAction;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
import frc.robot.util.ButtonWatcher;
import frc.robot.util.PoseUtils;
import java.util.function.Supplier;

public class AutoPlace {
    private static final Distance StartSuperStructureRange = Inches.of(20);
    private static final Distance StartSuperStructureRangeAlgae = Inches.of(20);

    // if player lets go of back buttons finish moving to pose but dont outtake
    public static Command autoAlignAndPlace(
            CommandXboxController driverController,
            Drive drive,
            SuperStructure superStructure,
            Intake intake,
            Side side) {
        Supplier<Pose2d> drivePose = () -> FieldConstants.Reef.offsetReefPose(
                drive.getPose().nearest(FieldConstants.Reef.getAllianceReefList()), side);
        ButtonWatcher buttonWatcher = new ButtonWatcher(driverController);
        // drive to reef, once level is selected
        return DriveCommands.driveToPose(drive, () -> drivePose.get())
                .alongWith(new SequentialCommandGroup(
                        buttonWatcher.WaitSelectPose(),
                        new WaitUntilCommand(() -> PoseUtils.distanceBetweenPoses(drive.getPose(), drivePose.get())
                                .lt(AutoPlace.StartSuperStructureRange)),
                        superStructure.moveToPose(buttonWatcher::getSelectedPose),
                        intake.run(IntakeAction.PLACING)
                                .withTimeout(IntakeConstants.PlacingTimeout)
                                .andThen(intake.run(IntakeAction.PLACING)
                                        .withTimeout(IntakeConstants.PlacingTimeout)
                                        .deadlineFor(superStructure.retractArm(ArmConstants.baseAngle)))));
    }

    public static Command autoAlignAndPickAlgaeL2(Drive drive, SuperStructure superStructure, AlgaeIntake algaeIntake) {
        Supplier<Pose2d> drivePose = () -> FieldConstants.Reef.offsetReefPose(
                drive.getPose().nearest(FieldConstants.Reef.getAllianceReefList()), Side.CENTER);
        // ButtonWatcher buttonWatcher = new ButtonWatcher(driverController);
        // drive to reef, once level is selected
        return DriveCommands.driveToPose(drive, () -> drivePose.get())
                .alongWith(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> PoseUtils.distanceBetweenPoses(drive.getPose(), drivePose.get())
                                .lt(AutoPlace.StartSuperStructureRangeAlgae)),
                        superStructure.moveToPose(SuperStructurePose.L2_ALGAE),
                        AlgaeCommands.removeL2AlgaeV2(superStructure, algaeIntake)));
    }

    public static Command autoAlignAndPickAlgaeL3(Drive drive, SuperStructure superStructure, AlgaeIntake algaeIntake) {
        Supplier<Pose2d> drivePose = () -> FieldConstants.Reef.offsetReefPose(
                drive.getPose().nearest(FieldConstants.Reef.getAllianceReefList()), Side.CENTER);
        // ButtonWatcher buttonWatcher = new ButtonWatcher(driverController);
        // drive to reef, once level is selected
        return DriveCommands.driveToPose(drive, () -> drivePose.get())
                .alongWith(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> PoseUtils.distanceBetweenPoses(drive.getPose(), drivePose.get())
                                .lt(AutoPlace.StartSuperStructureRangeAlgae)),
                        superStructure.moveToPose(SuperStructurePose.L3_ALGAE),
                        AlgaeCommands.removeL3AlgaeV2(superStructure, algaeIntake)));
    }

    public static Command autoScoreBarge(Drive drive, SuperStructure superStructure, AlgaeIntake algaeIntake) {
        Supplier<Pose2d> drivePose = () -> FieldConstants.Net.getNetPose(drive.getPose());
        return DriveCommands.driveToPose(drive, drivePose)
                .andThen(superStructure.moveToPose(SuperStructurePose.ALGAE_PRE_NET))
                .andThen(superStructure.moveToPose(SuperStructurePose.ALGAE_NET))
                .andThen(algaeIntake.run(AlgaeIntakeAction.NET).withTimeout(AlgaeIntakeConstants.PlacingTimeout));
    }
}
