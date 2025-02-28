package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

// would be nice to alway know what level we are at?
public class SuperStructure {
    public enum SuperStructurePose {
        // constants call the constructor

        // Elevator position, arm angle
        TROUGH(Inches.of(9.0), Degrees.of(-35)),
        LOADING(Inches.of(21.5), Degrees.of(-25)),
        L2(Inches.of(3), Degrees.of(142.5)),
        L3(Inches.of(19), Degrees.of(142.5)),
        L4(Inches.of(53.0), Degrees.of(170)),

        L2_ALGAE(Inches.of(0), Degrees.of(155)),
        L3_ALGAE(Inches.of(24), Degrees.of(155)),

        HOLD(Inches.of(3.0), Degrees.of(110)),
        BASE(Inches.of(18.0), Degrees.of(-25)); // 3.0, -25

        public final Distance elevatorPosition;
        public final Angle armAngle;

        SuperStructurePose(Distance elevatorPosition, Angle armAngle) {
            this.elevatorPosition = elevatorPosition;
            this.armAngle = armAngle;
        }
    }

    public final Elevator elevator;
    public final Arm arm;
    private SuperStructurePose currentPose;
    private SuperStructurePose targetPose = SuperStructurePose.BASE;

    public SuperStructure(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

    public void setTargetPose(Supplier<SuperStructurePose> pose) {
        targetPose = pose.get();
    }

    public SuperStructurePose getPose() {
        return currentPose;
    }

    public SuperStructurePose getTargetPose() {
        return targetPose;
    }

    private Angle getMovingAngle(SuperStructurePose pose) {
        if (pose == SuperStructurePose.L2 || pose == SuperStructurePose.L3 || pose == SuperStructurePose.L4) {
            return Degrees.of(135);
        } else {
            return pose.armAngle;
        }
    }

    public Command holdPose(Intake intake) {
        boolean holding = intake.hasCoral();
        return moveToPose(holding ? SuperStructurePose.HOLD : SuperStructurePose.BASE)
                .alongWith(intake.runRaw(
                        holding ? IntakeConstants.HoldingSpeed : RevolutionsPerSecond.of(0.0), Amps.of(20)));
    }

    // start moving arm to actual angle once we are within a certain
    // range to the desired elevator position
    public Command moveToPose(SuperStructurePose pose) {
        return elevator.moveToPosition(pose.elevatorPosition)
                .alongWith(new WaitUntilCommand(() -> pose.elevatorPosition
                                .minus(elevator.getPosition())
                                .lt(ElevatorConstants.retractingError))
                        .deadlineFor(arm.moveToAngle(getMovingAngle(pose)))
                        .andThen(arm.moveToAngle(pose.armAngle)))
                .beforeStarting(() -> targetPose = pose)
                .finallyDo(() -> {
                    currentPose = pose;
                });
    }

    public Command moveToPose(Supplier<SuperStructurePose> poseSupplier) {
        return Commands.run(() -> {}, elevator)
                .beforeStarting(() -> elevator.setPosition(poseSupplier.get().elevatorPosition))
                .until(() -> elevator.isAtPosition(poseSupplier.get().elevatorPosition))
                .alongWith(new WaitUntilCommand(() -> poseSupplier
                                .get()
                                .elevatorPosition
                                .minus(elevator.getPosition())
                                .lt(ElevatorConstants.retractingError))
                        .deadlineFor(Commands.run(() -> {}, arm)
                                .beforeStarting(() -> arm.setArmPosition(getMovingAngle(poseSupplier.get())))
                                .until(() -> arm.isArmAtPosition(getMovingAngle(poseSupplier.get()))))
                        .andThen(Commands.run(() -> {}, arm)
                                .beforeStarting(() -> arm.setArmPosition(poseSupplier.get().armAngle)))
                        .until(() -> arm.isArmAtPosition(poseSupplier.get().armAngle)))
                .beforeStarting(() -> targetPose = poseSupplier.get())
                .finallyDo(() -> currentPose = poseSupplier.get());
    }

    public BooleanSupplier waitTillRetracted() {
        return () -> arm.getArmPosition().minus(ArmConstants.safeRetractAngle).lt(Degrees.of(0));
    }

    public Command retractArm(Angle retractAngle) {
        return arm.moveToAngle(retractAngle);
    }
}
