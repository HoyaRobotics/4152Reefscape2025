// Auto drive to closest branch and move to pose indicated by buttons
// on remote, outtaking onto it
// if one of the level buttons is pressed before its ready it will take
// that into account and place with no delay

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef.Side;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
import frc.robot.util.PoseUtils;

public class AutoPlace {
    public static final Distance StartSuperStructureRange = Inches.of(12);

    // how do we know if a button has been pressed and what pose to use

    public static Command autoAlignAndPlace(
            CommandXboxController driverController,
            Drive drive,
            SuperStructure superStructure,
            Intake intake,
            Side side) {
        Supplier<SuperStructurePose> superStructurePose = () -> SuperStructurePose.BASE;
        Supplier<Pose2d> drivePose = () -> FieldConstants.Reef.offsetReefPose(
                drive.getPose().nearest(FieldConstants.Reef.getAllianceReefList()), side);
        // drive to reef, once level is selected
        return DriveCommands.driveToPose(drive, () -> drivePose.get())
                .alongWith(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> {
                            boolean selected = true;
                            if(driverController.a().getAsBoolean()){
                                superStructurePose = () -> SuperStructurePose.L2;
                            } else if (driverController.b().getAsBoolean()){
                                superStructurePose = () -> SuperStructurePose.TROUGH;
                            } else if (driverController.x().getAsBoolean()) {
                                superStructurePose = () -> SuperStructurePose.L3;
                            } else if (driverController.y().getAsBoolean()) {
                                superStructurePose = () -> SuperStructurePose.L4;
                            } else {
                                selected = false;
                            }

                            return selected;
                        }
                        ),
                        new WaitUntilCommand(() -> PoseUtils.distanceBetweenPoses(drive.getPose(), drivePose.get())
                                .lt(AutoPlace.StartSuperStructureRange)),
                        // figure out which pose to get
                        superStructure.moveToPosePreAngle(superStructurePose.get()),
                        intake.run(false)
                                .withTimeout(IntakeConstants.PlacingTimeout)
                                .andThen(intake.run(false)
                                        .withTimeout(IntakeConstants.PlacingTimeout)
                                        .deadlineFor(superStructure.retractArm(ArmConstants.baseAngle)))));
    }
}
