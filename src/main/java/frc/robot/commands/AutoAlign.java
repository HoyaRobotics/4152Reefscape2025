// Auto drive to closest branch and move to pose indicated by buttons
// on remote, outtaking onto it
// if one of the level buttons is pressed before its ready it will take
// that into account and place with no delay

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.DriveMap;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Processor;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeIntakeAction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeAction;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.util.ButtonWatcher;
import frc.robot.util.PoseUtils;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

public class AutoAlign {
    private static final Distance StartSuperStructureRange = Inches.of(45); // 20
    private static final Distance StartSuperStructureRangeAlgae = Inches.of(65);
    public static final Distance ThrowNetTolerance = Inches.of(14); // 12
    private static final double L2DelaySeconds = 0.075; // 0.125

    // if player lets go of back buttons finish moving to pose but dont outtake
    // switch while moving ifn ew level chosen
    public static Command autoAlignAndPlace(
            DriveMap driveController,
            Drive drive,
            SuperStructure superStructure,
            Intake intake,
            AlgaeIntake algaeIntake,
            Side side,
            Optional<SuperStructurePose> superStructurePose,
            boolean removeAlgae) {
        Supplier<Pose2d> drivePose = () -> Reef.getClosestBranchPose(drive, side);
        ButtonWatcher buttonWatcher = new ButtonWatcher(driveController);
        // drive to reef, once level is selected
        return Commands.sequence(
                        new DriveToPoseHeading(drive, drivePose::get, Optional.of(Degrees.of(360)))
                                .alongWith(Commands.sequence(
                                        buttonWatcher.WaitSelectPose().onlyIf(() -> superStructurePose.isEmpty()),
                                        new WaitUntilCommand(
                                                () -> PoseUtils.distanceBetweenPoses(drive.getPose(), drivePose.get())
                                                        .lt(AutoAlign.StartSuperStructureRange)),
                                        new DeferredCommand(
                                                () -> superStructure.moveToPose(
                                                        superStructurePose.orElse(buttonWatcher.getSelectedPose())),
                                                Set.of(superStructure.arm, superStructure.elevator)))),
                        placingSequence(
                                superStructure,
                                intake,
                                () -> superStructurePose.orElse(buttonWatcher.getSelectedPose()),
                                false),
                        autoAlignAndPickAlgae(drive, superStructure, algaeIntake)
                                .onlyIf(() -> removeAlgae))
                .beforeStarting(() -> buttonWatcher.selectedPose = Optional.empty())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command placingSequence(
            SuperStructure superStructure, Intake intake, Supplier<SuperStructurePose> currentPose, boolean isAuto) {
        return Commands.sequence(
                Commands.waitSeconds(L2DelaySeconds)
                        .onlyIf(() -> currentPose.get() == SuperStructurePose.L2
                                || currentPose.get() == SuperStructurePose.L3),
                intake.runWithSensor(IntakeAction.PLACING),
                Commands.either(
                                superStructure
                                        .arm
                                        .moveToAngle(SuperStructurePose.BASE.armAngle)
                                        .until(() -> superStructure.arm.isPastPosition(Degrees.of(130), false)),
                                superStructure.arm.moveToAngle(Degrees.of(103)),
                                () -> isAuto)
                        .deadlineFor(intake.run(IntakeAction.PLACING)));
    }

    public static Command autoAlignLoadProcessor(Drive drive, SuperStructure superStructure, AlgaeIntake algaeIntake) {
        Supplier<Pose2d> movingPose =
                () -> Processor.getProcessorPose().transformBy(new Transform2d(-0.35, 0.0, new Rotation2d()));
        Supplier<Pose2d> drivePose = () -> Processor.getProcessorPose();
        return superStructure
                .moveToPose(SuperStructurePose.PROCESSOR)
                .deadlineFor(new DriveToPose(drive, movingPose::get, Optional.of(Degrees.of(360))))
                .andThen(new DriveToPose(drive, drivePose::get, Optional.of(Degrees.of(360)))
                        .alongWith(Commands.sequence(
                                new WaitUntilCommand(
                                        () -> PoseUtils.distanceBetweenPoses(drive.getPose(), drivePose.get())
                                                .lt(AutoAlign.StartSuperStructureRange)),
                                superStructure.moveToPose(SuperStructurePose.PROCESSOR)))
                        .andThen(algaeIntake
                                .run(AlgaeIntakeAction.PROCESSOR)
                                .withTimeout(AlgaeIntakeConstants.PlacingTimeout)));
    }

    public static Command autoAlignAndPickAlgae(Drive drive, SuperStructure superStructure, AlgaeIntake algaeIntake) {
        Supplier<Pose2d> drivePose = () -> Reef.getClosestBranchPose(drive, Side.CENTER);
        Supplier<Pose2d> movingPose = () ->
                Reef.getClosestBranchPose(drive, Side.CENTER).transformBy(new Transform2d(0.15, 0.0, Rotation2d.kZero));

        return new DriveToPose(drive, movingPose::get, Optional.empty())
                .alongWith((AlgaeCommands.preStageRemoveAlgaeV2(superStructure, algaeIntake, drive)))
                .andThen(Commands.sequence(
                        new DriveToPose(drive, drivePose::get, Optional.empty()),
                        AlgaeCommands.removeAlgaeV2(superStructure, algaeIntake, drive),
                        new DriveToPose(drive, movingPose::get, Optional.empty()),
                        superStructure.arm.moveToAngle(Degrees.of(130))));
    }

    public static Command autoScoreBarge(Drive drive, SuperStructure superStructure, AlgaeIntake algaeIntake) {
        Supplier<Pose2d> drivePose = () -> FieldConstants.Net.getNetPose(drive.getPose(), Optional.empty());
        return new DriveToPose(drive, drivePose::get, Optional.of(Degrees.of(360)))
                .andThen(superStructure
                        .moveToPose(SuperStructurePose.ALGAE_NET)
                        .alongWith(Commands.waitUntil(() -> SuperStructurePose.ALGAE_NET
                                        .elevatorPosition
                                        .minus(superStructure.elevator.getPosition())
                                        .lt(ThrowNetTolerance))
                                .andThen(algaeIntake.run(AlgaeIntakeAction.NET))))
                .andThen(algaeIntake.run(AlgaeIntakeAction.NET).withTimeout(AlgaeIntakeConstants.PlacingTimeout));
    }
}
