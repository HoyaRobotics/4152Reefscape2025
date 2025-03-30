package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.HoldPosition;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import java.util.List;
import java.util.function.Supplier;

public class AlgaeCenter extends PoserAuto {
    public AlgaeCenter(
            Side autoSide,
            Drive drive,
            SuperStructure superStructure,
            Intake intake,
            AlgaeIntake algaeIntake,
            LED leds) {
        super(autoSide, drive, superStructure, intake, algaeIntake, leds);
    }

    public Command getAutoCommand() {
        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        List<Integer> reefFaces = List.of(3, 2, 1).stream()
                .map((index) -> sideRelativeIndex(index))
                .toList();

        List<Side> branchSides = List.of(Side.RIGHT).stream()
                .map((side) -> sideRelativeBranch(side))
                .toList();

        autoCommand.addCommands(alignAndPlaceCoral(SuperStructurePose.L4, reefFaces.get(0), branchSides.get(0), true));
        autoCommand.addCommands(alignAndPlaceBarge(Meters.of(0)));
        autoCommand.addCommands(alignAndPickAlgae(reefFaces.get(1)));
        autoCommand.addCommands(alignAndPlaceBarge(Meters.of(1)));

        Supplier<Pose2d> translationPoint = () -> {
            var branchPose = Reef.getAllianceReefBranch(reefFaces.get(2), Side.CENTER);

            return branchPose.transformBy(new Transform2d(1.0, 0.0, Rotation2d.kZero));
        };
        autoCommand.addCommands(transitionWaypoint(translationPoint, Meters.of(0.5))
                .deadlineFor(new HoldPosition(superStructure.elevator, superStructure.arm, intake, algaeIntake)));
        autoCommand.addCommands(alignAndPickAlgae(reefFaces.get(2)));
        autoCommand.addCommands(alignAndPlaceBarge(Meters.of(2)));

        return autoCommand;
    }
}
