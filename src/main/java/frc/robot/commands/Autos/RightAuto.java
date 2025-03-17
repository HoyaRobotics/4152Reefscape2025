package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAlign;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import java.util.function.Supplier;

public class RightAuto extends PoserAuto {

    public RightAuto(Drive drive, SuperStructure superStructure, Intake intake, AlgaeIntake algaeIntake) {
        super(drive, superStructure, intake, algaeIntake);
    }

    public Command getAutoCommand() {
        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        autoCommand.addCommands(alignAndPlaceCoral(4, Side.LEFT));

        Supplier<Pose2d> waypointPose =
                () -> Reef.getAllianceReefBranch(4, Side.LEFT).transformBy(new Transform2d(0.5, 0, Rotation2d.kZero));
        autoCommand.addCommands(transitionWaypoint(waypointPose, Meters.of(0.25))
                .deadlineFor(superStructure.moveToPose(SuperStructurePose.LOADING)));

        Supplier<Pose2d> secondWaypoint = () ->
                Reef.getAllianceReefBranch(5, Side.RIGHT).transformBy(new Transform2d(0.75, -0.5, Rotation2d.kZero));
        autoCommand.addCommands(transitionWaypoint(secondWaypoint, Meters.of(0.5))
                .deadlineFor(superStructure.moveToPose(SuperStructurePose.LOADING)));

        autoCommand.addCommands(alignAndReceiveCoral(Side.RIGHT));
        autoCommand.addCommands(alignAndPlaceCoral(5, Side.RIGHT));

        autoCommand.addCommands(alignAndReceiveCoral(Side.RIGHT));
        autoCommand.addCommands(alignAndPlaceCoral(5, Side.LEFT));
        autoCommand.addCommands(AutoAlign.autoAlignAndPickAlgae(drive, superStructure, algaeIntake));

        return autoCommand;
    }
}
