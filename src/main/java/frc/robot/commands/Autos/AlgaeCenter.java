package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.HoldPosition;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.subsystems.vision.Vision;
import java.util.List;

public class AlgaeCenter extends PoserAuto {
    public AlgaeCenter(
            Side autoSide,
            Drive drive,
            SuperStructure superStructure,
            Intake intake,
            AlgaeIntake algaeIntake,
            LED leds,
            Vision vision) {
        super(autoSide, drive, superStructure, intake, algaeIntake, leds, vision);
    }

    public Command getAutoCommand() {
        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        List<Integer> reefFaces =
                List.of(3, 4).stream().map((index) -> sideRelativeIndex(index)).toList();

        List<Side> branchSides = List.of(Side.RIGHT).stream()
                .map((side) -> sideRelativeBranch(side))
                .toList();

        // look at picking up algae before placing?

        autoCommand.addCommands(Commands.waitSeconds(0.25));
        autoCommand.addCommands(alignAndPlaceCoral(SuperStructurePose.L4, reefFaces.get(0), branchSides.get(0), true));
        autoCommand.addCommands(alignAndPlaceBarge(Meters.of(0)));
        autoCommand.addCommands(
                superStructure.moveToPose(SuperStructurePose.BASE).withTimeout(0.2));
        autoCommand.addCommands(alignAndPickAlgae(reefFaces.get(1)));
        // drive back to barge

        /*
        autoCommand.addCommands(
                AutoAlign.driveToPose(drive, () -> Net.getNetPose(drive.getPose(), Optional.of(Meters.of(1)))));*/
        autoCommand.addCommands(alignAndPlaceBarge(Meters.of(1)));
        autoCommand.addCommands(
                superStructure.moveToPose(SuperStructurePose.BASE).withTimeout(0.2));

        // start driving towards coral station
        autoCommand.addCommands(AutoAlign.driveToPose(
                        drive,
                        () -> new Pose2d(
                                Reef.getAllianceReefBranch(reefFaces.get(1), Side.CENTER)
                                        .transformBy(new Transform2d(1.25, 1.5, Rotation2d.k180deg))
                                        .getTranslation(),
                                Rotation2d.kZero))
                .deadlineFor(new HoldPosition(superStructure.elevator, superStructure.arm, intake, algaeIntake)));

        return autoCommand;
    }
}
