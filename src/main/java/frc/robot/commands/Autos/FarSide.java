package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAlign;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.subsystems.vision.Vision;
import java.util.function.Supplier;

public class FarSide extends PoserAuto {
    public FarSide(
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

        Pose2d lanePose = new Pose2d(2.5, 7.2, Rotation2d.kZero);
        Pose2d transitionPose = new Pose2d(1.28, 5.1, Rotation2d.kZero);

        Supplier<Pose2d> targetPose = () -> {
            Pose2d relativePose = drive.getPose().relativeTo(lanePose);

            return relativePose.getMeasureX().lt(Inches.of(0.5)) ? transitionPose : lanePose;
        };

        Supplier<Pose2d> intakePose = () -> new Pose2d();

        autoCommand.addCommands(AutoAlign.driveToPose(drive, targetPose));
        autoCommand.addCommands(alignAndPlaceCoral(SuperStructurePose.L4, 3, Side.LEFT, false));
        autoCommand.addCommands(AutoAlign.driveToPose(drive, () -> transitionPose));
        // autoCommand.addcommands(alignAndReceiveCoral());

        return autoCommand;
    }
}
