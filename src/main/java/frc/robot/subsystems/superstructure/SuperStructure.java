package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import java.util.Collections;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

// would be nice to alway know what level we are at?
public class SuperStructure {

    public enum AlgaeLevel {
        ALGAE_L2,
        ALGAE_L3
    }

    public enum SuperStructurePose {
        // constants call the constructor

        // Elevator position, arm angle
        TROUGH(Inches.of(7.0), Degrees.of(-60)),
        LOADING(Inches.of(18), Degrees.of(-35)),
        PROCESSOR(Inches.of(0.0), Degrees.of(0)),
        L2(Inches.of(13.0), Degrees.of(135.0)),
        L3(Inches.of(28.75), Degrees.of(135.0)), // 15.75 more than L2
        L4(Inches.of(52.75), Degrees.of(140.0)),

        L2_ALGAE(Inches.of(4.0), Degrees.of(170)),
        L2_ALGAE_GRAB(Inches.of(8.5), Degree.of(170)),
        L2_ALGAE_REMOVE(Inches.of(0.0), Degree.of(103)),
        L3_ALGAE(Inches.of(19.75), Degrees.of(170)), // 15.75 more than L2 20.75
        L3_ALGAE_GRAB(Inches.of(24.25), Degrees.of(170)), // 15.75 more than L2 25.75
        L3_ALGAE_REMOVE(Inches.of(9.25), Degrees.of(103)), // 15.75 more than L2 10.75

        ALGAE_PRE_NET(Inches.of(52.75), Degrees.of(103)),
        ALGAE_NET(Inches.of(52.75), Degrees.of(83)),

        HOLD(Inches.of(3.0), Degrees.of(103)),
        CLIMB_STOW(Inches.of(0.0), Degrees.of(-15)),
        BASE(Inches.of(18), Degrees.of(-35)); // 3.0, -25

        public final Distance elevatorPosition;
        public final Angle armAngle;

        SuperStructurePose(Distance elevatorPosition, Angle armAngle) {
            this.elevatorPosition = elevatorPosition;
            this.armAngle = armAngle;
        }
    }

    public static List<SuperStructurePose> getAlgaePoses(AlgaeLevel algaeLevel) {
        switch (algaeLevel) {
            case ALGAE_L2:
                return List.of(
                        SuperStructurePose.L2_ALGAE,
                        SuperStructurePose.L2_ALGAE_GRAB,
                        SuperStructurePose.L2_ALGAE_REMOVE);
            case ALGAE_L3:
                return List.of(
                        SuperStructurePose.L3_ALGAE,
                        SuperStructurePose.L3_ALGAE_GRAB,
                        SuperStructurePose.L3_ALGAE_REMOVE);
            default:
                return Collections.emptyList();
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
            return Degrees.of(103);
        } else {
            return pose.armAngle;
        }
    }

    // start moving arm to actual angle once we are within a certain
    // range to the desired elevator position
    public Command moveToPose(SuperStructurePose pose) {
        return elevator.moveToPosition(pose.elevatorPosition)
                .alongWith(new WaitUntilCommand(() -> pose.elevatorPosition
                                .minus(elevator.getPosition())
                                .lt(ElevatorConstants.tiltingDistance))
                        .deadlineFor(arm.moveToAngle(getMovingAngle(pose)))
                        .andThen(arm.moveToAngle(pose.armAngle)))
                .beforeStarting(() -> targetPose = pose)
                .finallyDo(() -> {
                    currentPose = pose;
                });
    }

    public boolean isAtPosition() {
        return elevator.isAtPosition(targetPose.elevatorPosition) && arm.isArmAtPosition(targetPose.armAngle);
    }

    public BooleanSupplier waitTillRetracted() {
        return () -> arm.getArmPosition().minus(ArmConstants.safeRetractAngle).lt(Degrees.of(0));
    }

    public Command retractArm(Angle retractAngle) {
        return arm.moveToAngle(retractAngle);
    }
}
