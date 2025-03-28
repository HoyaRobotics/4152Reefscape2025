package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveToPoseHeading;
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

    public PoserAuto(Drive drive, SuperStructure superStructure, Intake intake, AlgaeIntake algaeIntake) {
        this.drive = drive;
        this.superStructure = superStructure;
        this.intake = intake;
        this.algaeIntake = algaeIntake;
    }

    public Command alignAndPlaceCoral(SuperStructurePose superStructurePose, int reefFaceIndex, Side side) {
        Supplier<Pose2d> targetPose = () -> Reef.getAllianceReefBranch(reefFaceIndex, side);
        return new DriveToPoseHeading(drive, targetPose, Optional.of(PlacingAngleDeltaTolerance))
                .alongWith(Commands.defer(
                                () -> Commands.waitUntil(
                                        PoseUtils.poseInRange(drive::getPose, targetPose, PlacingDistance)),
                                Set.of())
                        .deadlineFor(new HoldPosition(superStructure.elevator, superStructure.arm, intake, algaeIntake))
                        .andThen(superStructure.moveToPose(superStructurePose)))
                .andThen(superStructure
                        .moveToPose(superStructurePose)
                        .andThen(AutoAlign.placingSequence(superStructure, intake, () -> superStructurePose, true)));
    }

    public Command transitionWaypoint(Supplier<Pose2d> targetPose, Distance tolerance) {
        return new DriveToPoseHeading(drive, targetPose, Optional.of(Degrees.of(360)))
                .until(PoseUtils.poseInRange(drive::getPose, targetPose, tolerance));
    }

    public Command alignAndReceiveCoral(Side side) {
        Supplier<Pose2d> targetPose = () -> CoralStation.getCoralStationPose(side);
        return intake.runWithSensor(IntakeAction.INTAKING)
                .deadlineFor(new DriveToPoseHeading(drive, targetPose, Optional.of(CoralStationAngleDelta))
                        .alongWith(superStructure
                                .arm
                                .moveToAngle(SuperStructurePose.BASE.armAngle)
                                .until(() -> superStructure.arm.isPastPosition(Degrees.of(130), false))
                                .andThen(superStructure.moveToPose(SuperStructurePose.LOADING))))
                .andThen(() -> intake.stopIntake());
    }
}
