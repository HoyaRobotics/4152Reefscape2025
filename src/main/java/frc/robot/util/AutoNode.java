package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.Supplier;

public class AutoNode {
    private Supplier<Pose2d> poseSupplier;

    private Optional<Angle> angleDeltaTolerance;
    private Optional<Pair<Distance, Angle>> transitionTolerance;
    private Command toSchedule;
    private Command deadlineGroup;

    public AutoNode(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        angleDeltaTolerance = Optional.empty();
        transitionTolerance = Optional.empty();
        toSchedule = Commands.none();
        deadlineGroup = Commands.none();
    }

    public AutoNode(Pose2d targetPose) {
        this(() -> targetPose);
    }

    public Pose2d getTargetPose() {
        return poseSupplier.get();
    }

    public Command getCommand(Drive drive, RobotContainer robot) {
        return new DriveToPose(drive, poseSupplier, angleDeltaTolerance, Optional.empty(), true, false)
                .asProxy()
                .until(
                        transitionTolerance.isPresent()
                                ? PoseUtils.poseInRange(
                                        drive::getPose,
                                        poseSupplier.get(),
                                        transitionTolerance.get().getFirst())
                                : () -> false)
                .deadlineFor(deadlineGroup)
                .alongWith(toSchedule);
    }

    public AutoNode addDeadline(Command deadline) {
        deadlineGroup = deadline;
        return this;
    }

    public AutoNode setCommand(Command toSchedule) {
        this.toSchedule = toSchedule;
        return this;
    }

    public AutoNode addCommand(Command toSchedule) {
        this.toSchedule = this.toSchedule.alongWith(toSchedule);
        return this;
    }

    public AutoNode addCommandOnInRange(Supplier<Pose2d> poseSupplier, Command toSchedule, Distance triggerRange) {
        this.addCommand(Commands.waitUntil(PoseUtils.poseInRange(poseSupplier, this.poseSupplier.get(), triggerRange))
                .andThen(toSchedule.beforeStarting(() -> System.out.println("Command in range"))));
        return this;
    }

    public AutoNode addCommandOnInRange(
            Supplier<Pose2d> poseSupplier, Command toSchedule, Command whileWaiting, Distance triggerRange) {
        this.addCommand(Commands.waitUntil(PoseUtils.poseInRange(poseSupplier, this.poseSupplier.get(), triggerRange))
                .deadlineFor(whileWaiting)
                .andThen(toSchedule.beforeStarting(() -> System.out.println("Command in range"))));
        return this;
    }

    public AutoNode setDeltaTolerance(Angle angleDeltaTolerance) {
        this.angleDeltaTolerance = Optional.of(angleDeltaTolerance);
        return this;
    }

    public AutoNode setTransitionTolerance(Distance linearTolerance, Angle angleTolerance) {
        this.transitionTolerance = Optional.of(new Pair<Distance, Angle>(linearTolerance, angleTolerance));
        return this;
    }
}
