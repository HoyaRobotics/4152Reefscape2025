// Auto drive to closest branch and move to pose indicated by buttons
// on remote, outtaking onto it
// if one of the level buttons is pressed before its ready it will take
// that into account and place with no delay

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
import frc.robot.util.ButtonWatcher;
import frc.robot.util.PoseUtils;
import java.util.function.Supplier;

public class AutoPlace {
    private static final Distance StartSuperStructureRange = Inches.of(20);

    public static Command autoAlignAndPlace(
            CommandXboxController driverController,
            Drive drive,
            SuperStructure superStructure,
            Intake intake,
            Side side) {
        Supplier<Pose2d> drivePose = () -> FieldConstants.Reef.offsetReefPose(
                drive.getPose().nearest(FieldConstants.Reef.getAllianceReefList()), side);
        ButtonWatcher buttonWatcher = new ButtonWatcher(driverController);
        // drive to reef, once level is selected
        return DriveCommands.driveToPose(drive, () -> drivePose.get())
                .alongWith(new SequentialCommandGroup(
                        buttonWatcher.WaitSelectPose(),
                        new WaitUntilCommand(() -> PoseUtils.distanceBetweenPoses(drive.getPose(), drivePose.get())
                                .lt(AutoPlace.StartSuperStructureRange)),
                        superStructure.moveToPose(buttonWatcher::getSelectedPose),
                        intake.run(false)
                                .withTimeout(IntakeConstants.PlacingTimeout)
                                .andThen(intake.run(false)
                                        .withTimeout(IntakeConstants.PlacingTimeout)
                                        .deadlineFor(superStructure.retractArm(ArmConstants.baseAngle)))));
    }
}
