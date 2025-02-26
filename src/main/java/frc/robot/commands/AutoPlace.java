// Auto drive to closest branch and move to pose indicated by buttons
// on remote, outtaking onto it
// if one of the level buttons is pressed before its ready it will take
// that into account and place with no delay

package frc.robot.commands;

/*
public class AutoPlace {
    public static final Distance StartSuperStructureRange = Inches.of(12);

    // how do we know if a button has been pressed and what pose to use
    // need a way to know when one of the buttons has been pressed, not trigger the normal
    // move to level commands and save which pose we are going to move to

    // labelled driver mapping would be nice
    // and we will need to make sure neither of the align buttons are down
    // when triggering normal superstructure move commands

    // seperate singleton class that will return the last pose indicated by level buttons?
    /*
    private Command SelectPoseCommand(SuperStructure superStructure) {
        return new SelectCommand<>(
        Map.ofEntries(
                Map.entry(SuperStructurePose.BASE, superStructure.moveToPosePreAngle(SuperStructurePose.L2)),
                Map.entry
        )
    }


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
                        // figure out which pose to get
                        Commands.runOnce(() -> superStructure.setTargetPose(buttonWatcher::getSelectedPose)),
                        superStructure.moveToPosePreAngle(),
                        intake.run(false)
                                .withTimeout(IntakeConstants.PlacingTimeout)
                                .andThen(intake.run(false)
                                        .withTimeout(IntakeConstants.PlacingTimeout)
                                        .deadlineFor(superStructure.retractArm(ArmConstants.baseAngle)))));
    }
}

*/
