package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import java.util.List;

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

        List<Integer> reefFaces =
                List.of(3, 4).stream().map((index) -> sideRelativeIndex(index)).toList();

        List<Side> branchSides = List.of(Side.RIGHT).stream()
                .map((side) -> sideRelativeBranch(side))
                .toList();

        // look at picking up algae before placing?

        autoCommand.addCommands(Commands.waitSeconds(0.5));
        autoCommand.addCommands(alignAndPlaceCoral(SuperStructurePose.L3, reefFaces.get(0), branchSides.get(0), true));
        autoCommand.addCommands(alignAndPlaceBarge(Meters.of(0)));
        autoCommand.addCommands(alignAndPickAlgae(reefFaces.get(1)));
        /*
        autoCommand.addCommands(alignAndPlaceBarge(Meters.of(1)));*

        // start driving towards coral station
        autoCommand.addCommands(AutoAlign.driveToPose(
                        drive, () -> Reef.getAllianceReefBranch(reefFaces.get(1), Side.CENTER)
                                .transformBy(new Transform2d(1.25, 1.5, Rotation2d.kZero)))
                .deadlineFor(new HoldPosition(superStructure.elevator, superStructure.arm, intake, algaeIntake)));*/

        return autoCommand;
    }
}
