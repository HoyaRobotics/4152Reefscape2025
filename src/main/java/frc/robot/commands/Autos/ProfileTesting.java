package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoseProfiled;
import frc.robot.commands.HoldPosition;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ProfileTesting extends PoserAuto {
    public ProfileTesting(
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

        autoCommand.addCommands(alignAndPlaceCoral(SuperStructurePose.L4, 4, Side.LEFT, false));

        Supplier<Pose2d> targetPose = () -> {
            // once we are within y tolerance switch
            Pose2d branchPose = Reef.getAllianceReefBranch(4, autoSide);
            Pose2d coralStationPose = CoralStation.getCoralStationPose(autoSide);
            Pose2d transitionPose = branchPose.transformBy(new Transform2d(
                    0.25,
                    autoSide == Side.RIGHT ? -2.75 : 2.75,
                    branchPose.interpolate(coralStationPose, 0.5).getRotation()));

            Pose2d currentPose = drive.getPose();
            double tolerance = 1.75;

            Logger.recordOutput("Testing/transitionPose", transitionPose);
            Logger.recordOutput(
                    "Testing/yOffset", currentPose.relativeTo(transitionPose).getX() + tolerance);
            boolean pastTransitionTolerance =
                    currentPose.relativeTo(transitionPose).getX() + tolerance > 0;

            return pastTransitionTolerance ? coralStationPose : transitionPose;
        };

        autoCommand.addCommands(new DriveToPoseProfiled(drive, targetPose, Optional.empty())
                .deadlineFor(new HoldPosition(superStructure.elevator, superStructure.arm, intake, algaeIntake)));

        return autoCommand;
    }
}
