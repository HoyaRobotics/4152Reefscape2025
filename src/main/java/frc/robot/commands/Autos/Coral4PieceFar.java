package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import java.util.function.Supplier;

public class Coral4PieceFar extends PoserAuto {

    public Coral4PieceFar(Drive drive, SuperStructure superStructure, Intake intake, AlgaeIntake algaeIntake) {
        super(drive, superStructure, intake, algaeIntake);
    }

    public Command getAutoCommand(Side autoSide) {
        // coral object class / enum?, superstructure pose, branch side and reef face
        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        final int reefFace1 = autoSide == Side.RIGHT ? 4 : 2;
        final int reefFace2 = autoSide == Side.RIGHT ? 5 : 1;

        final Side branchSide1 = autoSide == Side.RIGHT ? Side.RIGHT : Side.LEFT;
        final Side branchSide2 = autoSide == Side.RIGHT ? Side.RIGHT : Side.LEFT;
        final Side branchSide3 = autoSide == Side.RIGHT ? Side.LEFT : Side.RIGHT;
        final Side branchSide4 = autoSide == Side.RIGHT ? Side.LEFT : Side.RIGHT;

        autoCommand.addCommands(alignAndPlaceCoral(reefFace1, branchSide1));

        Supplier<Pose2d> waypointPose1 = () -> Reef.getAllianceReefBranch(reefFace1, branchSide1)
                .transformBy(new Transform2d(0.25, autoSide == Side.RIGHT ? -1.5 : 1.5, Rotation2d.kZero));
        autoCommand.addCommands(transitionWaypoint(waypointPose1, Meters.of(0.5))
                .deadlineFor(superStructure.moveToPose(SuperStructurePose.LOADING)));

        autoCommand.addCommands(alignAndReceiveCoral(autoSide));
        autoCommand.addCommands(alignAndPlaceCoral(reefFace2, branchSide2));

        autoCommand.addCommands(alignAndReceiveCoral(autoSide));
        autoCommand.addCommands(alignAndPlaceCoral(reefFace2, branchSide3));

        autoCommand.addCommands(alignAndReceiveCoral(autoSide));

        Supplier<Pose2d> waypointPose2 = () -> Reef.getAllianceReefBranch(reefFace1, branchSide1)
                .transformBy(new Transform2d(0.25, autoSide == Side.RIGHT ? -1.5 : 1.5, Rotation2d.kZero));
        autoCommand.addCommands(transitionWaypoint(waypointPose2, Meters.of(0.5)));

        autoCommand.addCommands(alignAndPlaceCoral(reefFace1, branchSide4));
        // autoCommand.addCommands(AutoAlign.autoAlignAndPickAlgae(drive, superStructure, algaeIntake));

        return autoCommand;
    }
}
