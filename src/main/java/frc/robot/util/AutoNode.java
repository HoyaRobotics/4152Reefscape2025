package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.Supplier;

public class AutoNode {
    private Supplier<Pose2d> poseSupplier;

    private Optional<Angle> angleDeltaTolerance;
    private Optional<Pair<Distance, Angle>> transitionTolerance;
    private Optional<Command> toSchedule;

    public AutoNode(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        angleDeltaTolerance = Optional.empty();
        transitionTolerance = Optional.empty();
        toSchedule = Optional.empty();
    }

    public AutoNode(Pose2d targetPose) {
        this(() -> targetPose);
    }

    public Pose2d getTargetPose() {
        return poseSupplier.get();
    }

    public Command getCommand(Drive drive) {
        return new DriveToPose(drive, poseSupplier, angleDeltaTolerance, Optional.empty(), true)
                .until(
                        transitionTolerance.isPresent()
                                ? PoseUtils.poseInRange(
                                        drive::getPose,
                                        poseSupplier.get(),
                                        transitionTolerance.get().getFirst())
                                : () -> false)
                .alongWith(toSchedule.orElse(Commands.none()));
        // return new DriveToPose(drive, () -> this.targetPose, angleDeltaTolerance, transitionTolerance, true)
        //        .alongWith(toSchedule.orElse(Commands.none()));
    }

    public AutoNode setCommand(Command toSchedule) {
        this.toSchedule = Optional.of(toSchedule);
        return this;
    }

    public AutoNode setDeltaTolerance(Angle angleDeltaTolerance) {
        this.angleDeltaTolerance = Optional.of(angleDeltaTolerance);
        return this;
    }

    public AutoNode setTransitionTolerance(Distance linearTolerance, Angle angleTolerance) {
        this.transitionTolerance = Optional.of(new Pair<>(linearTolerance, angleTolerance));
        return this;
    }
}
