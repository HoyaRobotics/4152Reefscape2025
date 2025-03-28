package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.HoldPosition;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeAction;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.util.PoseUtils;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

// each auto node includes: target pose, tolerance, list of command, distance pairs which will schedule the
// command the provided distance away from the next pose
public abstract class PoserAuto {

    static final Distance PlacingDistance = Inches.of(70); // 35
    static final Angle PlacingAngleDeltaTolerance = Degrees.of(360);

    static final Angle CoralStationAngleDelta = Degrees.of(360);

    protected final Drive drive;
    protected final SuperStructure superStructure;
    protected final Intake intake;
    protected final AlgaeIntake algaeIntake;

    protected final Side autoSide;

    public PoserAuto(
            Side autoSide, Drive drive, SuperStructure superStructure, Intake intake, AlgaeIntake algaeIntake) {
        this.autoSide = autoSide;
        this.drive = drive;
        this.superStructure = superStructure;
        this.intake = intake;
        this.algaeIntake = algaeIntake;
    }
    // instead of adding waypoint poses, just change the pose in the drive to pose supplier,
    // make it a moving pose?

    // place barge at given pose
    protected int sideRelativeIndex(int rightFaceIndex) {
        return autoSide == Side.RIGHT
                ? rightFaceIndex
                : switch (rightFaceIndex) {
                    case 0 -> 0;
                    case 1 -> 5;
                    case 2 -> 4;
                    case 3 -> 3;
                    case 4 -> 2;
                    case 5 -> 1;
                    default -> 3;
                };
    }

    protected Side sideRelativeBranch(Side rightBranchSide) {
        return autoSide == Side.RIGHT
                ? rightBranchSide
                : switch (rightBranchSide) {
                    case RIGHT -> Side.LEFT;
                    case LEFT -> Side.RIGHT;
                    case CENTER -> Side.CENTER;
                    default -> Side.CENTER;
                };
    }

    public Command alignAndPickAlgae(int faceIndex) {
        return AutoAlign.autoAlignAndPickAlgae(drive, superStructure, algaeIntake, Optional.of(faceIndex));
    }

    public Command alignAndPlaceBarge(Distance bargeCenterOffset) {
        return AutoAlign.autoScoreBarge(
                drive, superStructure, algaeIntake, Optional.of(bargeCenterOffset), () -> 0.0, () -> 0.0);
    }

    public Command alignAndPlaceCoral(
            SuperStructurePose superStructurePose, int reefFaceIndex, Side side, boolean removeAlgae) {
        Supplier<Pose2d> targetPose = () -> Reef.getAllianceReefBranch(reefFaceIndex, side);
        return Commands.sequence(
                new DriveToPose(drive, targetPose)
                        .alongWith(Commands.defer(
                                        () -> Commands.waitUntil(
                                                PoseUtils.poseInRange(drive::getPose, targetPose, PlacingDistance)),
                                        Set.of())
                                .deadlineFor(new HoldPosition(
                                        superStructure.elevator, superStructure.arm, intake, algaeIntake))
                                .andThen(superStructure.moveToPose(superStructurePose))),
                superStructure.moveToPose(superStructurePose),
                AutoAlign.placingSequence(superStructure, intake, () -> superStructurePose, true),
                AutoAlign.autoAlignAndPickAlgae(drive, superStructure, algaeIntake, Optional.of(reefFaceIndex))
                        .onlyIf(() -> removeAlgae));
    }

    public Command transitionWaypoint(Supplier<Pose2d> targetPose, Distance tolerance) {
        return new DriveToPose(drive, targetPose).until(PoseUtils.poseInRange(drive::getPose, targetPose, tolerance));
    }

    public Command alignAndReceiveCoral(Side side) {
        Supplier<Pose2d> targetPose = () -> CoralStation.getCoralStationPose(side);
        return intake.runWithSensor(IntakeAction.INTAKING)
                .deadlineFor(new DriveToPose(drive, targetPose)
                        .alongWith(superStructure
                                .arm
                                .moveToAngle(SuperStructurePose.BASE.armAngle)
                                .until(() -> superStructure.arm.isPastPosition(Degrees.of(130), false))
                                .andThen(superStructure.moveToPose(SuperStructurePose.LOADING))))
                .andThen(() -> intake.stopIntake());
    }
}
