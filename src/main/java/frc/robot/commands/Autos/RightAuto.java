package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.HoldPosition;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.intake.IntakeConstants.IntakeAction;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.util.AutoNode;
import java.util.LinkedList;
import java.util.List;

// variable intake height proportional to distance
public class RightAuto extends PoserAuto {
    @Override
    protected List<AutoNode> getAutoGraph(RobotContainer robot) {
        List<AutoNode> nodes = new LinkedList();

        // place preloaded
        nodes.add(new AutoNode(() -> Reef.getAllianceReefBranch(4, Side.LEFT))
                .setDeltaTolerance(Degrees.of(360))
                .addCommandOnInRange(
                        robot.drive::getPose,
                        AutoAlign.placingSequence(robot.superStructure, robot.intake, SuperStructurePose.L4),
                        Inches.of(55)));

        // Go get first coral
        nodes.add(new AutoNode(() -> Reef.getAllianceReefBranch(4, Side.LEFT)
                        .transformBy(new Transform2d(0.0, -2.75, new Rotation2d())))
                .addDeadline(robot.superStructure.moveToPose(SuperStructurePose.LOADING))
                .setDeltaTolerance(Degrees.of(360))
                .setTransitionTolerance(Meters.of(2.0), Degrees.of(360)));
        nodes.add(new AutoNode(() -> CoralStation.getCoralStationPose(Side.RIGHT))
                .setCommand(robot.intake
                        .runWithSensor(IntakeAction.INTAKING)
                        .deadlineFor(robot.superStructure.moveToPose(SuperStructurePose.LOADING))
                        .andThen(() -> robot.intake.stopIntake()))
                .setDeltaTolerance(Degrees.of(360)));

        // place second coral
        nodes.add(new AutoNode(Reef.getAllianceReefBranch(5, Side.RIGHT))
                .setDeltaTolerance(Degrees.of(360))
                .addCommandOnInRange(
                        robot.drive::getPose,
                        AutoAlign.placingSequence(robot.superStructure, robot.intake, SuperStructurePose.L4),
                        new HoldPosition(robot.elevator, robot.arm, robot.intake, robot.algaeIntake),
                        Inches.of(55)));

        // get third coral
        nodes.add(new AutoNode(() -> CoralStation.getCoralStationPose(Side.RIGHT))
                .setCommand(robot.intake
                        .runWithSensor(IntakeAction.INTAKING)
                        .deadlineFor(robot.superStructure.moveToPose(SuperStructurePose.LOADING))
                        .andThen(() -> robot.intake.stopIntake()))
                .setDeltaTolerance(Degrees.of(360)));

        // place third coral
        nodes.add(new AutoNode(Reef.getAllianceReefBranch(5, Side.LEFT))
                .setDeltaTolerance(Degrees.of(360))
                .addCommandOnInRange(
                        robot.drive::getPose,
                        AutoAlign.placingSequence(robot.superStructure, robot.intake, SuperStructurePose.L4),
                        new HoldPosition(robot.elevator, robot.arm, robot.intake, robot.algaeIntake),
                        Inches.of(55)));
        // get tfourth coral
        nodes.add(new AutoNode(() -> CoralStation.getCoralStationPose(Side.RIGHT))
                .setCommand(robot.intake
                        .runWithSensor(IntakeAction.INTAKING)
                        .deadlineFor(robot.superStructure.moveToPose(SuperStructurePose.LOADING))
                        .andThen(() -> robot.intake.stopIntake()))
                .setDeltaTolerance(Degrees.of(360)));
        return nodes;
    }
}
