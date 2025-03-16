package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.util.AutoNode;
import java.util.List;
import org.littletonrobotics.junction.Logger;

// each auto node includes: target pose, tolerance, list of command, distance pairs which will schedule the
// command the provided distance away from the next pose
public abstract class PoserAuto {

    public Command getAutoCommand(RobotContainer robot) {
        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        List<AutoNode> nodes = getAutoGraph(robot);
        nodes.forEach((node) -> autoCommand.addCommands(node.getCommand(robot.drive, robot)));
        List<Pose2d> autoPoses =
                nodes.stream().map((node) -> node.getTargetPose()).toList();
        Logger.recordOutput("PIDAuto/Poses", autoPoses.toArray(new Pose2d[0]));

        return autoCommand;
    }

    protected abstract List<AutoNode> getAutoGraph(RobotContainer robot);
}
