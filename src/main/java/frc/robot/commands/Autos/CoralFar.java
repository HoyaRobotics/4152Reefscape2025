package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.function.Supplier;

public class CoralFar extends PoserAuto {

    public CoralFar(
            Side autoSide,
            Drive drive,
            SuperStructure superStructure,
            Intake intake,
            AlgaeIntake algaeIntake,
            LED leds) {
        super(autoSide, drive, superStructure, intake, algaeIntake, leds);
    }

    public Command getAutoCommand() {
        // coral object class / enum?, superstructure pose, branch side and reef face
        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        final List<Integer> reefFaces =
                List.of(4, 5).stream().map((index) -> sideRelativeIndex(index)).toList();

        final List<Side> branchSides = List.of(Side.RIGHT, Side.RIGHT, Side.LEFT, Side.LEFT).stream()
                .map((side) -> sideRelativeBranch(side))
                .toList();

        autoCommand.addCommands(alignAndPlaceCoral(SuperStructurePose.L4, reefFaces.get(0), branchSides.get(0), false));

        Supplier<Pose2d> targetPose = () -> {
            Pose2d branchPose = Reef.getAllianceReefBranch(reefFaces.get(0), autoSide);
            Pose2d coralStationPose = CoralStation.getCoralStationPose(autoSide);
            Pose2d transitionPose = branchPose.transformBy(new Transform2d(
                    0.25,
                    autoSide == Side.RIGHT ? -2.75 : 2.75,
                    branchPose.interpolate(coralStationPose, 0.5).getRotation()));

            Pose2d currentPose = drive.getPose();
            double tolerance = 1.75;

            boolean pastTransitionTolerance =
                    currentPose.relativeTo(transitionPose).getX() + tolerance > 0;

            return pastTransitionTolerance
                    ? coralStationPose.transformBy(
                            new Transform2d(0.0, autoSide == Side.RIGHT ? 0.5 : -0.5, Rotation2d.kZero))
                    : transitionPose;
        };

        autoCommand.addCommands(alignAndReceiveCoral(targetPose));
        autoCommand.addCommands(alignAndPlaceCoral(SuperStructurePose.L4, reefFaces.get(1), branchSides.get(1), false));

        autoCommand.addCommands(alignAndReceiveCoral(autoSide));
        autoCommand.addCommands(alignAndPlaceCoral(SuperStructurePose.L4, reefFaces.get(1), branchSides.get(2), false));

        autoCommand.addCommands(alignAndReceiveCoral(autoSide));

        Supplier<Pose2d> targetPose2 = () -> {
                Pose2d branchPose = Reef.getAllianceReefBranch(reefFaces.get(0), autoSide);
                Pose2d transitionPose = branchPose.transformBy(new Transform2d(
                        0.25,
                        autoSide == Side.RIGHT ? 0.0 : -0.0,
                        Rotation2d.kZero));

                return transitionPose;
        };

        autoCommand.addCommands(transitionWaypoint(targetPose2, Meters.of(0.5)));
        autoCommand.addCommands(alignAndPlaceCoral(SuperStructurePose.L4, reefFaces.get(0), branchSides.get(3), false));

        return autoCommand;
    }
}
