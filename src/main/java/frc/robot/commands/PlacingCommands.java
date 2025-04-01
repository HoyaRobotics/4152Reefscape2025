package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeAction;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.leds.LED.LEDState;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class PlacingCommands {
    private static final double L2DelaySeconds = 0.125;

    public static Command reefPlacingSequence(
            SuperStructure superStructure,
            Intake intake,
            LED leds,
            Supplier<SuperStructurePose> currentPose,
            boolean isAuto) {
        return Commands.sequence(
                        Commands.waitSeconds(L2DelaySeconds)
                                .onlyIf(() -> currentPose.get() == SuperStructurePose.L2
                                        || currentPose.get() == SuperStructurePose.L3),
                        intake.runWithSensor(IntakeAction.PLACING),
                        Commands.either(
                                        superStructure
                                                .arm
                                                .moveToAngle(SuperStructurePose.BASE.armAngle)
                                                .until(() -> superStructure.arm.isPastPosition(Degrees.of(130), false)),
                                        superStructure.arm.moveToAngle(Degrees.of(103)),
                                        () -> isAuto)
                                .deadlineFor(intake.run(IntakeAction.PLACING)))
                .deadlineFor(Commands.startEnd(
                        () -> leds.requestState(LEDState.PLACING), () -> leds.requestState(LEDState.NOTHING)));
    }

    public static Command troughPlacingSequence(
            SuperStructure superStructure, Intake intake, BooleanSupplier placeObject) {
        return Commands.sequence(
                superStructure.moveToPose(SuperStructurePose.TROUGH),
                Commands.waitUntil(placeObject),
                intake.runWithSensor(IntakeAction.TROUGH),
                intake.run(IntakeAction.TROUGH).withDeadline(superStructure.moveToPose(SuperStructurePose.POST_TROUGH)),
                Commands.waitSeconds(IntakeConstants.PostPlacingTimeout),
                superStructure.moveToPose(SuperStructurePose.POST_POST_TROUGH));
    }
}
