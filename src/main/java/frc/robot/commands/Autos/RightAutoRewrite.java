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

public class RightAutoRewrite extends PoserAuto {

    public RightAutoRewrite(Drive drive, SuperStructure superStructure, Intake intake, AlgaeIntake algaeIntake) {
        super(drive, superStructure, intake, algaeIntake);
    }

    public Command getAutoCommand() {
        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        autoCommand.addCommands(alignAndPlaceCoral(4, Side.LEFT));

        Supplier<Pose2d> waypointPose =
                () -> Reef.getAllianceReefBranch(4, Side.LEFT).transformBy(new Transform2d(0.5, 0, new Rotation2d()));
        autoCommand.addCommands(transitionWaypoint(waypointPose, Meters.of(0.25))
                .deadlineFor(superStructure.moveToPose(SuperStructurePose.L4)));

        Supplier<Pose2d> secondWaypoint = () -> Reef.getAllianceReefBranch(5, Side.RIGHT)
            .transformBy(new Transform2d(0.5, 0, new Rotation2d()));
        autoCommand.addCommands(transitionWaypoint(secondWaypoint, Meters.of(0.25)));

        autoCommand.addCommands(alignAndReceiveCoral(Side.RIGHT));
        autoCommand.addCommands(alignAndPlaceCoral(5, Side.RIGHT));

        autoCommand.addCommands(alignAndReceiveCoral(Side.RIGHT));
        autoCommand.addCommands(alignAndPlaceCoral(5, Side.LEFT));

        return autoCommand;
    }
}
