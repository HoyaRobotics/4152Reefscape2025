package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.util.AutoNode;
import java.util.List;

import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly.CoralStationsSide;

public class RightAuto extends PoserAuto {
    @Override
    protected List<AutoNode> getAutoGraph(RobotContainer robot) {
        List<AutoNode> nodes = List.of(
                new AutoNode(Reef.offsetReefPose(Reef.redCenterFaces[4], Side.LEFT))
                        .setDeltaTolerance(Degrees.of(360))
                        .setCommand(robot.superStructure.moveToPose(SuperStructurePose.L4)),
                new AutoNode(Reef.offsetReefPose(Reef.redCenterFaces[4], Side.LEFT)
                        .transformBy(new Transform2d(0.0, -1.5, new Rotation2d())))
                        .setDeltaTolerance(Degrees.of(360))
                        .setTransitionTolerance(Meters.of(1.5), Degrees.of(360)),
                new AutoNode(() -> CoralStation.getCoralStationPose(Side.RIGHT))
                        .setDeltaTolerance(Degrees.of(360)));

        return nodes;
    }
}
