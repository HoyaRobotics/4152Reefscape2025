// Auto drive to closest branch and move to pose indicated by buttons
// on remote, outtaking onto it
// if one of the level buttons is pressed before its ready it will take
// that into account and place with no delay

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.DriveMap;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef;
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
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

public class AutoAlign {
    private static final Distance StartSuperStructureRange = Inches.of(20);
    private static final Distance StartSuperStructureRangeAlgae = Inches.of(50);

    // if player lets go of back buttons finish moving to pose but dont outtake
    // switch while moving ifn ew level chosen
    public static Command autoAlignAndPlace(
            DriveMap driveController,
            Drive drive,
            SuperStructure superStructure,
            Intake intake,
            Side side,
            Optional<SuperStructurePose> superStructurePose) {
        Supplier<Pose2d> drivePose = () -> Reef.getClosestBranchPose(drive, side);
        ButtonWatcher buttonWatcher = new ButtonWatcher(driveController);
        // drive to reef, once level is selected
        return new DriveToPose(drive, drivePose::get, Optional.of(Degrees.of(50)))
                .alongWith(new SequentialCommandGroup(
                        new DeferredCommand(
                                () -> superStructurePose.isEmpty() ? buttonWatcher.WaitSelectPose() : Commands.none(),
                                Set.of(superStructure.arm, superStructure.elevator)),
                        new WaitUntilCommand(() -> PoseUtils.distanceBetweenPoses(drive.getPose(), drivePose.get())
                                .lt(AutoAlign.StartSuperStructureRange)),
                        new DeferredCommand(
                                () -> superStructure.moveToPose(
                                        superStructurePose.isEmpty()
                                                ? buttonWatcher.getSelectedPose()
                                                : superStructurePose.get()),
                                Set.of(superStructure.arm, superStructure.elevator))))
                .andThen(intake.run(IntakeAction.PLACING)
                        .withTimeout(IntakeConstants.PlacingTimeout)
                        .andThen(intake.run(IntakeAction.PLACING)
                                .withTimeout(IntakeConstants.PlacingTimeout)
                                .deadlineFor(superStructure.retractArm(ArmConstants.baseAngle))));
    }

    public static Command autoAlignAndPickAlgae(Drive drive, SuperStructure superStructure, AlgaeIntake algaeIntake) {
        Supplier<Pose2d> drivePose = () -> Reef.getClosestBranchPose(drive, Side.CENTER);
        return new DriveToPose(drive, drivePose::get, Optional.empty())
                .alongWith(new WaitUntilCommand(() -> PoseUtils.distanceBetweenPoses(drive.getPose(), drivePose.get())
                                .lt(AutoAlign.StartSuperStructureRangeAlgae))
                        .andThen(AlgaeCommands.preStageRemoveAlgaeV2(superStructure, algaeIntake, drive)))
                .andThen(AlgaeCommands.removeAlgaeV2(superStructure, algaeIntake, drive));
    }

    public static Command autoScoreBarge(Drive drive, SuperStructure superStructure, AlgaeIntake algaeIntake) {
        Supplier<Pose2d> drivePose = () -> FieldConstants.Net.getNetPose(drive.getPose());
        return new DriveToPose(drive, drivePose::get, Optional.of(Degrees.of(360)))
                .andThen(superStructure.moveToPose(SuperStructurePose.ALGAE_PRE_NET))
                .andThen(superStructure.moveToPose(SuperStructurePose.ALGAE_NET))
                .andThen(algaeIntake.run(AlgaeIntakeAction.NET).withTimeout(AlgaeIntakeConstants.PlacingTimeout))
                .andThen(superStructure.moveToPose(SuperStructurePose.HOLD));
    }
}
