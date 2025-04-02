// Auto drive to closest branch and move to pose indicated by buttons
// on remote, outtaking onto it
// if one of the level buttons is pressed before its ready it will take
// that into account and place with no delay

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.DriveMap;
import frc.robot.commands.DriveToPose.DriveToPoseProfiled;
import frc.robot.commands.DriveToPose.DriveToPoseRaw;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Processor;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeIntakeAction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeAction;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.leds.LED.LEDState;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.util.ButtonWatcher;
import frc.robot.util.LockableSupplier;
import frc.robot.util.PoseUtils;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AutoAlign {
    private static final Distance StartSuperStructureRange = Inches.of(45); // 20
    private static final Distance StartSuperStructureRangeAlgae = Inches.of(65);
    public static final Distance ThrowNetTolerance = Inches.of(14); // 12
    public static final double HorizontalVelocityPredictionTolerance = 2.5; // m/s
    public static final LinearVelocity HorizontalVelocityRisingTolerance = MetersPerSecond.of(2.0); // m/s
    public static final Angle AngleDifferenceRisingTolerance = Degrees.of(30);
    public static final Distance LockingDistance = Meters.of(0.75);

    public static Command driveToPose(Drive drive, Supplier<Pose2d> targetPose) {
        return Commands.either(
                new DriveToPoseProfiled(drive, targetPose),
                new DriveToPoseRaw(drive, targetPose),
                () -> DriverStation.isAutonomous() ? Constants.AutoMotionProfiling : Constants.TeleopMotionProfiling);
    }

    public static Command driveToPose(Drive drive, Supplier<Pose2d> targetPose, Supplier<Translation2d> linearFF) {
        return Commands.either(
                new DriveToPoseProfiled(drive, targetPose, linearFF),
                new DriveToPoseRaw(drive, targetPose, linearFF),
                () -> DriverStation.isAutonomous() ? Constants.AutoMotionProfiling : Constants.TeleopMotionProfiling);
    }

    // allow movement along coral station plane
    public static Command alignAndReceiveCoral(
            Drive drive, SuperStructure superStructure, LED leds, Intake intake, DoubleSupplier inputY) {
        Supplier<Pose2d> targetPose = () -> {
            Pose2d closestStation =
                    CoralStation.offsetCoralStationPose(CoralStation.getClosestCoralStation(drive.getPose()));

            double yOffset = drive.getPose().relativeTo(closestStation).getY();
            yOffset = Math.max(-0.5, Math.min(0.5, yOffset));
            // clamp y to go to closest position
            return closestStation.transformBy(new Transform2d(0.0, yOffset, Rotation2d.kZero));
        };
        return intake.run(IntakeAction.INTAKING)
                .alongWith(driveToPose(
                                drive,
                                targetPose,
                                () -> DriveCommands.getLinearVelocityFromJoysticks(0.0, inputY.getAsDouble()))
                        .deadlineFor((Commands.either(
                                superStructure.moveToLoadingPose(drive),
                                superStructure.moveToPose(SuperStructurePose.LOADING),
                                () -> Constants.useVariableIntakeHeight))));
    }

    public static Command autoAlignAndPlace(
            DriveMap driveController,
            Drive drive,
            SuperStructure superStructure,
            Intake intake,
            Side side,
            AlgaeIntake algaeIntake,
            LED leds,
            DoubleSupplier inputX,
            DoubleSupplier inputY) {

        ButtonWatcher buttonWatcher = new ButtonWatcher(driveController);
        buttonWatcher.selectedPose = Optional.of(SuperStructurePose.L4);

        LockableSupplier<Pose2d> targetPose = new LockableSupplier<>(() -> {
            var reefFaces = Reef.getAllianceReefList();
            var closestFace = drive.getPose().nearest(reefFaces);
            var closestIndex = reefFaces.indexOf(closestFace);

            var horizontalVelocity = DriveCommands.getTargetRelativeLinearVelocity(drive, closestFace)
                    .getY();

            if (Math.abs(horizontalVelocity) < HorizontalVelocityPredictionTolerance
                    || drive.getPose()
                                    .getTranslation()
                                    .getDistance(Reef.offsetReefPose(closestFace, side)
                                            .getTranslation())
                            < LockingDistance.in(Meters)) return Reef.offsetReefPose(closestFace, side);

            return horizontalVelocity > 0
                    ? Reef.getAllianceReefBranch(closestIndex == 5 ? 0 : closestIndex + 1, side)
                    : Reef.getAllianceReefBranch(closestIndex == 0 ? 5 : closestIndex - 1, side);
        });

        return Commands.sequence(
                        driveToPose(drive, targetPose)
                                .beforeStarting(() -> targetPose.lock())
                                .alongWith(Commands.sequence(
                                        Commands.waitUntil(
                                                        () -> PoseUtils.poseInRange(
                                                                        drive::getPose,
                                                                        targetPose,
                                                                        StartSuperStructureRange)
                                                                && (buttonWatcher.getSelectedPose()
                                                                                == SuperStructurePose.L2
                                                                        || buttonWatcher.getSelectedPose()
                                                                                == SuperStructurePose.L3
                                                                        || (Math.abs(DriveCommands
                                                                                                .getTargetRelativeLinearVelocity(
                                                                                                        drive,
                                                                                                        targetPose
                                                                                                                .get())
                                                                                                .getY())
                                                                                        < HorizontalVelocityRisingTolerance
                                                                                                .in(MetersPerSecond)
                                                                                && drive.getPose()
                                                                                        .getRotation()
                                                                                        .getMeasure()
                                                                                        .isNear(
                                                                                                targetPose
                                                                                                        .get()
                                                                                                        .getRotation()
                                                                                                        .getMeasure(),
                                                                                                AngleDifferenceRisingTolerance)))),
                                        Commands.defer(
                                                () -> superStructure.moveToPose(buttonWatcher.getSelectedPose())
                                                        .beforeStarting(() -> leds.requestState(LEDState.PLACING)),
                                                Set.of(superStructure.arm, superStructure.elevator)))),
                        PlacingCommands.reefPlacingSequence(
                                superStructure, intake, leds, () -> buttonWatcher.getSelectedPose(), false))
                .deadlineFor(Commands.startEnd(
                        () -> leds.requestState(LEDState.ALIGNING), () -> leds.requestState(LEDState.NOTHING)))
                .finallyDo(() -> targetPose.unlock())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command autoAlignLoadProcessor(
            Drive drive, SuperStructure superStructure, LED leds, AlgaeIntake algaeIntake) {
        Supplier<Pose2d> movingPose =
                () -> Processor.getProcessorPose(drive).transformBy(new Transform2d(-0.35, 0.0, new Rotation2d()));
        Supplier<Pose2d> drivePose = () -> Processor.getProcessorPose(drive);
        return superStructure
                .moveToPose(SuperStructurePose.PROCESSOR)
                .deadlineFor(driveToPose(drive, movingPose::get))
                .andThen(driveToPose(drive, drivePose::get)
                        .alongWith(Commands.sequence(
                                Commands.waitUntil(() -> PoseUtils.poseInRange(
                                        drive::getPose, drivePose, StartSuperStructureRangeAlgae)),
                                superStructure.moveToPose(SuperStructurePose.PROCESSOR)))
                        .andThen(algaeIntake
                                .run(AlgaeIntakeAction.PROCESSOR)
                                .withTimeout(AlgaeIntakeConstants.PlacingTimeout))
                        .beforeStarting(() -> leds.requestState(LEDState.PLACING)))
                .deadlineFor(Commands.startEnd(
                        () -> leds.requestState(LEDState.ALIGNING), () -> leds.requestState(LEDState.NOTHING)));
    }

    public static Command autoAlignAndPickAlgae(
            Drive drive,
            SuperStructure superStructure,
            LED leds,
            AlgaeIntake algaeIntake,
            Optional<Integer> faceIndex) {
        Supplier<Pose2d> grabPose = () -> faceIndex
                .map(index -> Reef.getAllianceReefBranch(index, Side.CENTER))
                .orElse(Reef.getClosestBranchPose(drive, Side.CENTER));
        Supplier<Pose2d> offsetPose = () -> grabPose.get().transformBy(new Transform2d(0.15, 0.0, Rotation2d.kZero));
        return Commands.sequence(
                        AlgaeCommands.preStageRemoveAlgaeV2(superStructure, algaeIntake, drive)
                                .deadlineFor(driveToPose(drive, offsetPose)),
                        driveToPose(drive, grabPose),
                        driveToPose(drive, offsetPose))
                /*
                AlgaeCommands.removeAlgaeV2(superStructure, algaeIntake, drive)
                        .deadlineFor(Commands.startEnd(
                                () -> leds.requestState(LEDState.PLACING),
                                () -> leds.requestState(LEDState.ALIGNING))),
                driveToPose(drive, offsetPose).alongWith(superStructure.arm.moveToAngle(Degrees.of(130))))*/
                .deadlineFor(Commands.startEnd(
                        () -> leds.requestState(LEDState.ALIGNING), () -> leds.requestState(LEDState.NOTHING)));
    }

    public static Command autoScoreBarge(
            Drive drive,
            SuperStructure superStructure,
            AlgaeIntake algaeIntake,
            LED leds,
            Optional<Distance> bargeCenterOffset,
            DoubleSupplier inputY) {
        Supplier<Pose2d> drivePose = () -> FieldConstants.Net.getNetPose(drive.getPose(), bargeCenterOffset);
        return Commands.sequence(
                        driveToPose(
                                drive,
                                drivePose::get,
                                () -> DriveCommands.getLinearVelocityFromJoysticks(0.0, inputY.getAsDouble())),
                        superStructure
                                .moveToPose(SuperStructurePose.ALGAE_NET)
                                .deadlineFor(Commands.waitUntil(() -> SuperStructurePose.ALGAE_NET
                                                .elevatorPosition
                                                .minus(superStructure.elevator.getPosition())
                                                .lt(ThrowNetTolerance))
                                        .andThen(algaeIntake.run(AlgaeIntakeAction.NET))
                                        .beforeStarting(() -> leds.requestState(LEDState.PLACING))),
                        algaeIntake.run(AlgaeIntakeAction.NET).withTimeout(AlgaeIntakeConstants.PlacingTimeout))
                .deadlineFor(Commands.startEnd(
                        () -> leds.requestState(LEDState.ALIGNING), () -> leds.requestState(LEDState.NOTHING)));
    }
}
