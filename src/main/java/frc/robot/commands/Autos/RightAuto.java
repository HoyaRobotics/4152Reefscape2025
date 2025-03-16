package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAlign;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.subsystems.intake.IntakeConstants.IntakeAction;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.util.AutoNode;
import java.util.List;

public class RightAuto extends PoserAuto {
    @Override
    protected List<AutoNode> getAutoGraph(RobotContainer robot) {
        List<AutoNode> nodes = List.of(
                new AutoNode(Reef.offsetReefPose(Reef.redCenterFaces[4], Side.LEFT))
                        .setDeltaTolerance(Degrees.of(360))
                        .addCommandOnInRange(
                                robot.drive::getPose,
                                AutoAlign.placingSequence(robot.superStructure, robot.intake, SuperStructurePose.L4),
                                Inches.of(60)),
                new AutoNode(Reef.offsetReefPose(Reef.redCenterFaces[4], Side.LEFT)
                                .transformBy(new Transform2d(2.0, -0.5, new Rotation2d())))
                        .setDeltaTolerance(Degrees.of(360))
                        .setTransitionTolerance(Meters.of(1.75), Degrees.of(360)),
                new AutoNode(() -> CoralStation.getCoralStationPose(Side.RIGHT))
                        .setCommand(robot.intake
                                .runWithSensor(IntakeAction.INTAKING)
                                .deadlineFor(robot.superStructure.moveToPose(SuperStructurePose.LOADING))
                                .andThen(() -> robot.intake.stopIntake()))
                        .setDeltaTolerance(Degrees.of(360)),
                new AutoNode(Reef.offsetReefPose(Reef.redCenterFaces[5], Side.RIGHT))
                        .setDeltaTolerance(Degrees.of(360)),
                new AutoNode(() -> CoralStation.getCoralStationPose(Side.RIGHT)).setDeltaTolerance(Degrees.of(360)),
                new AutoNode(Reef.offsetReefPose(Reef.redCenterFaces[5], Side.LEFT)).setDeltaTolerance(Degrees.of(360)),
                new AutoNode(() -> CoralStation.getCoralStationPose(Side.RIGHT)).setDeltaTolerance(Degrees.of(360)));

        return nodes;
    }
}
