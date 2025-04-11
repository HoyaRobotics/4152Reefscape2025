// Auto drive to closest branch and move to pose indicated by buttons
// on remote, outtaking onto it
// if one of the level buttons is pressed before its ready it will take
// that into account and place with no delay

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import frc.robot.constants.FieldConstants.StagingPositions;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ButtonWatcher;
import frc.robot.util.LockableSupplier;
import frc.robot.util.PoseUtils;

public class AutoAlign {
    private static final Distance StartSuperStructureRange = Inches.of(45); // 20
    private static final Distance StartSuperStructureRangeAlgae = Inches.of(65);
    public static final Distance ThrowNetTolerance = Inches.of(8); // 12
    public static final double HorizontalVelocityPredictionTolerance = 2.5; // m/s
    public static final LinearVelocity HorizontalVelocityRisingTolerance = MetersPerSecond.of(2.0); // m/s
    public static final Angle AngleDifferenceRisingTolerance = Degrees.of(30);
    public static final Distance LockingDistance = Meters.of(0.75);
    public static final Angle BargeThrowAngleTolerance = Degrees.of(8);
    public static final Angle BargeRaisingRotTolerance = Degrees.of(10);
    public static final Distance ProcessorThrowTolerance = Inches.of(5.0);
    public static final Angle ProcessorRotTolerance = Degrees.of(8);
    public static final Distance BargeDistanceTolerance = Inches.of(5.0);

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

    public static Command alignGetStagedAlgae(
            Drive drive, SuperStructure superStructure, AlgaeIntake algaeIntake, LED leds) {
        LockableSupplier<Pose2d> stagingPose = new LockableSupplier<>(() -> {
            Pose2d currentPose = drive.getPose();
            var algaePose = currentPose.nearest(StagingPositions.getAllianceStartingAlgaePoses());

            var rotationOffset = new Rotation2d(Math.atan2(
                    currentPose.getTranslation().getY()
                            - algaePose.getTranslation().getY(),
                    currentPose.getTranslation().getX()
                            - algaePose.getTranslation().getX()));

            return new Pose2d(algaePose.getTranslation(), rotationOffset.plus(Rotation2d.fromDegrees(180)))
                    .transformBy(new Transform2d(-0.25, 0.0, Rotation2d.kZero));
        });

        Supplier<Pose2d> movingPose =
                () -> stagingPose.get().transformBy(new Transform2d(-0.75, 0.0, Rotation2d.kZero));

        return driveToPose(drive, movingPose)
                .until(() -> PoseUtils.poseInRange(drive::getPose, movingPose, Inches.of(0.5))
                        && drive.getPose()
                                .getRotation()
                                .getMeasure()
                                .isNear(movingPose.get().getRotation().getMeasure(), Degrees.of(5)))
                .alongWith(superStructure.moveToPose(SuperStructurePose.STAGED_ALGAE))
                .andThen(driveToPose(drive, stagingPose))
                .withDeadline(algaeIntake
                        .runWithSensor(AlgaeIntakeAction.INTAKING)
                        .andThen(algaeIntake.run(AlgaeIntakeAction.INTAKING).withTimeout(0.2)))
                .deadlineFor(Commands.startEnd(
                        () -> leds.requestState(LEDState.ALIGNING), () -> leds.requestState(LEDState.NOTHING)))
                .deadlineFor(Commands.startEnd(() -> stagingPose.lock(), () -> stagingPose.unlock()));
    }

    // allow movement along coral station plane
    public static Command alignAndReceiveCoral(
            Drive drive, SuperStructure superStructure, LED leds, Intake intake, DoubleSupplier inputY, Vision vision) {
        Supplier<Pose2d> targetPose = () -> {
            Pose2d closestStation =
                    CoralStation.offsetCoralStationPose(CoralStation.getClosestCoralStation(drive.getPose()));

            double yOffset = drive.getPose().relativeTo(closestStation).getY();
            yOffset = Math.max(-0.5, Math.min(0.5, yOffset));
            // clamp y to go to closest position
            return closestStation.transformBy(new Transform2d(0.0, yOffset, Rotation2d.kZero));
        };
        return intake.runWithSensor(IntakeAction.INTAKING)
                .deadlineFor(driveToPose(
                                drive,
                                targetPose,
                                () -> DriveCommands.getLinearVelocityFromJoysticks(0.0, inputY.getAsDouble()))
                        .alongWith(Commands.either(
                                superStructure.moveToLoadingPose(drive, vision),
                                superStructure.moveToPose(SuperStructurePose.LOADING),
                                () -> Constants.useVariableIntakeHeight)))
                .deadlineFor(Commands.startEnd(
                        () -> leds.requestState(LEDState.ALIGNING), () -> leds.requestState(LEDState.NOTHING)));
    }

    public static Command autoAlignAndPlace(
            Drive drive,
            SuperStructure superStructure,
            SuperStructurePose superStructurePose,
            Intake intake,
            LED leds,
            DoubleSupplier inputY) {

        final double SelectSideMagnitude = 0.5;

        Supplier<Side> closestBranch = () -> {
                var currentPose = drive.getPose();
                var closestFace = currentPose.nearest(Reef.getAllianceReefList());

                boolean rightCloser = currentPose.getTranslation()
                        .getDistance(Reef.offsetReefPose(closestFace, Side.RIGHT).getTranslation())
                        < currentPose.getTranslation().getDistance(Reef.offsetReefPose(closestFace, Side.LEFT).getTranslation());
                return rightCloser ? Side.RIGHT : Side.LEFT;
        };

        final Side[] branchSide = { closestBranch.get() };

        Supplier<Pose2d> targetPose = () -> {
            // update branch side based on user input
            double yScale = inputY.getAsDouble();

            var closestFace = drive.getPose()
                .nearest(Reef.getAllReefLists());
            var closestIndex = Reef.getAllianceReefList()
                .indexOf(closestFace);

            // faces 4, 3 and 2 are on the far side from the driver
            // this will be different from the drivers perspective depending on
            // what side of the reef we are on, just check using the closest face index
            if (Math.abs(yScale) > SelectSideMagnitude) {
                // we are on the far side of the reef
                if (closestIndex >= 2 && closestIndex <= 4) {
                        branchSide[0] = yScale > 0 ? Side.RIGHT : Side.LEFT;
                } else {
                        branchSide[0] = yScale > 0 ? Side.LEFT : Side.RIGHT;
                }
            }

            return Reef.offsetReefPose(drive.getPose().nearest(Reef.getAllianceReefList()), branchSide[0]);
        };

        // reduce drive velocity based on elevator height

        return autoAlignAndPlace(drive, superStructure, intake, leds, () -> superStructurePose, targetPose);
    }

    public static Command autoAlignAndPlace(
            DriveMap driveController,
            Drive drive,
            SuperStructure superStructure,
            Intake intake,
            Side side,
            LED leds,
            DoubleSupplier inputX,
            DoubleSupplier inputY) {

        ButtonWatcher buttonWatcher = new ButtonWatcher(driveController);
        buttonWatcher.selectedPose = Optional.of(SuperStructurePose.L4);

        Supplier<Pose2d> targetPose =
                () -> Reef.offsetReefPose(drive.getPose().nearest(Reef.getAllianceReefList()), side);

        return autoAlignAndPlace(drive, superStructure, intake, leds, buttonWatcher::getSelectedPose, targetPose);
    }

    public static Command autoAlignAndPlace(
            Drive drive,
            SuperStructure superStructure,
            Intake intake,
            LED leds,
            Supplier<SuperStructurePose> superPose,
            Supplier<Pose2d> targetPose) {
        BooleanSupplier startSuperStructure = () -> {
            var targetSuperPose = superPose.get();
            var finalPose = targetPose.get();
            final boolean superstructureLow =
                    targetSuperPose == SuperStructurePose.L2 || targetSuperPose == SuperStructurePose.L3;

            final boolean horizontalVelocitySlow =
                    Math.abs(DriveCommands.getTargetRelativeLinearVelocity(drive, finalPose)
                                    .getY())
                            < HorizontalVelocityRisingTolerance.in(MetersPerSecond);

            final boolean mostlyRotated = drive.getPose()
                    .getRotation()
                    .getMeasure()
                    .isNear(finalPose.getRotation().getMeasure(), AngleDifferenceRisingTolerance);

            return PoseUtils.poseInRange(drive::getPose, targetPose, StartSuperStructureRange)
                    && (superstructureLow || (horizontalVelocitySlow && mostlyRotated));
        };
        return Commands.sequence(
                        driveToPose(drive, targetPose)
                                .alongWith(Commands.sequence(
                                        Commands.waitUntil(startSuperStructure),
                                        Commands.defer(
                                                () -> superStructure.moveToPose(superPose.get()),
                                                Set.of(superStructure.arm, superStructure.elevator)))),
                        Commands.runOnce(() -> leds.requestState(LEDState.PLACING)),
                        PlacingCommands.reefPlacingSequence(superStructure, intake, leds, superPose, false))
                // some kind of wait condition to minimize jerkiness when picking algae
                // autoAlignAndPickAlgae(drive, superStructure, leds, algaeIntake, Optional.empty()))
                .deadlineFor(Commands.startEnd(
                        () -> leds.requestState(LEDState.ALIGNING), () -> leds.requestState(LEDState.NOTHING)))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command autoAlignLoadProcessor(
            Drive drive, SuperStructure superStructure, LED leds, AlgaeIntake algaeIntake) {
        Supplier<Pose2d> movingPose =
                () -> Processor.getProcessorPose(drive).transformBy(new Transform2d(-0.35, 0.0, new Rotation2d()));
        Supplier<Pose2d> drivePose = () -> Processor.getProcessorPose(drive);
        return superStructure
                .moveToPose(SuperStructurePose.PROCESSOR)
                .andThen(Commands.waitUntil(() -> drive.getPose()
                        .getRotation()
                        .getMeasure()
                        .isNear(movingPose.get().getRotation().getMeasure(), ProcessorRotTolerance)))
                .deadlineFor(driveToPose(drive, movingPose::get))
                .andThen(driveToPose(drive, drivePose::get)
                        .alongWith(Commands.waitUntil(
                                        () -> PoseUtils.poseInRange(drive::getPose, drivePose, ProcessorThrowTolerance))
                                .andThen(Commands.runOnce(() -> leds.requestState(LEDState.PLACING))
                                        .andThen(algaeIntake
                                                .run(AlgaeIntakeAction.PROCESSOR)
                                                .withTimeout(AlgaeIntakeConstants.PlacingTimeout))))
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
                .deadlineFor(algaeIntake.run(AlgaeIntakeAction.INTAKING))
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
                                        () -> DriveCommands.getLinearVelocityFromJoysticks(0.0, inputY.getAsDouble()))
                                .alongWith(Commands.waitUntil(() ->
                                                PoseUtils.poseInRange(drive::getPose, drivePose, BargeDistanceTolerance)
                                                        && drive.getPose()
                                                                .getRotation()
                                                                .getMeasure()
                                                                .isNear(
                                                                        drivePose
                                                                                .get()
                                                                                .getRotation()
                                                                                .getMeasure(),
                                                                        BargeRaisingRotTolerance))
                                        .andThen(superStructure
                                                .elevator
                                                .moveToPosition(SuperStructurePose.ALGAE_NET.elevatorPosition, true)
                                                .deadlineFor(Commands.waitUntil(() -> SuperStructurePose.ALGAE_NET
                                                                .elevatorPosition
                                                                .minus(superStructure.elevator.getPosition())
                                                                .lt(ThrowNetTolerance))
                                                        .andThen(superStructure
                                                                .arm
                                                                .moveToAngle(SuperStructurePose.ALGAE_NET.armAngle)
                                                                .alongWith(Commands.waitUntil(() ->
                                                                                superStructure.arm.withinTolerance(
                                                                                        SuperStructurePose.ALGAE_NET
                                                                                                .armAngle,
                                                                                        Degrees.of(8)))
                                                                        .andThen(algaeIntake.run(
                                                                                AlgaeIntakeAction.NET))))))),
                        algaeIntake.run(AlgaeIntakeAction.NET).withTimeout(AlgaeIntakeConstants.PlacingTimeout))
                .deadlineFor(Commands.startEnd(
                        () -> leds.requestState(LEDState.ALIGNING), () -> leds.requestState(LEDState.NOTHING)));
    }
}
