package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.Meters;

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
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Coral3Piece extends PoserAuto {

    public Coral3Piece(
            Side autoSide, Drive drive, SuperStructure superStructure, Intake intake, AlgaeIntake algaeIntake) {
        super(autoSide, drive, superStructure, intake, algaeIntake);
    }

    public Command getAutoCommand() {
        // coral object class / enum?, superstructure pose, branch side and reef face
        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        final List<Integer> reefFaces = List.of(4, 5, 0)
            .stream().map((index) -> sideRelativeIndex(index)).toList();

        final List<Side> branchSides = List.of(Side.LEFT, Side.RIGHT, Side.LEFT, Side.RIGHT)
            .stream().map((side) -> sideRelativeBranch(side)).toList();

        autoCommand.addCommands(alignAndPlaceCoral(SuperStructurePose.L4, reefFaces.get(0), branchSides.get(0), false));

        Supplier<Pose2d> waypointPose = () -> {
            var branchPose = Reef.getAllianceReefBranch(reefFaces.get(0), branchSides.get(0));
            var coralStationPose = CoralStation.getCoralStationPose(autoSide);
            var finalPose =  new Pose2d(
                    branchPose
                            .transformBy(new Transform2d(0.25, autoSide == Side.RIGHT ? -2.75 : 2.75, Rotation2d.kZero))
                            .getTranslation(),
                    branchPose.interpolate(coralStationPose, 0.5).getRotation());
            Logger.recordOutput("Auto/WayPoint", finalPose);
            return finalPose;
        };


        autoCommand.addCommands(transitionWaypoint(waypointPose, Meters.of(1.75))
                .deadlineFor(superStructure.moveToPose(SuperStructurePose.LOADING)));

        autoCommand.addCommands(alignAndReceiveCoral(autoSide));
        autoCommand.addCommands(alignAndPlaceCoral(SuperStructurePose.L4, reefFaces.get(1), branchSides.get(1), false));

        autoCommand.addCommands(alignAndReceiveCoral(autoSide));
        autoCommand.addCommands(alignAndPlaceCoral(SuperStructurePose.L4, reefFaces.get(1), branchSides.get(2), false));

        autoCommand.addCommands(alignAndReceiveCoral(autoSide));
        autoCommand.addCommands(alignAndPlaceCoral(SuperStructurePose.L4, reefFaces.get(2), branchSides.get(3), false));

        return autoCommand;
    }
}
