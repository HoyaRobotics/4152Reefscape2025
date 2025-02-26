package frc.robot.subsystems.superstructure;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import edu.wpi.first.units.measure.Angle;



// would be nice to alway know what level we are at?
public class SuperStructure {
    public enum SuperStructurePose {
        // constants call the constructor
        
        // Elevator position, arm angle
        TROUGH(Inches.of(9.0), Degrees.of(-35)),
        LOADING(Inches.of(20.5), Degrees.of(-21.5)),
        L2(Inches.of(3), Degrees.of(142.5)),
        L3(Inches.of(19), Degrees.of(142.5)),
        L4(Inches.of(53.0), Degrees.of(170)),

        L2_ALGAE(Inches.of(10), Degrees.of(165)),
        L3_ALGAE(Inches.of(20), Degrees.of(165)),

        HOLD(Inches.of(0.0), Degrees.of(110));


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

    public SuperStructure(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

    public SuperStructurePose getCurrentPose() {
        return currentPose;
    }

    private Angle getMovingAngle(SuperStructurePose pose) {
        if (pose == SuperStructurePose.L2 
            || pose == SuperStructurePose.L3
            || pose == SuperStructurePose.L4) {
            return Degrees.of(135);
        } else {
            return pose.armAngle;
        }
    }

    public Command moveToPose(SuperStructurePose pose) {
        return elevator.moveToPosition(pose.elevatorPosition)
            .alongWith(arm.moveToAngle(pose.armAngle))
            .finallyDo(() -> { currentPose = pose; });
    }

    public Command moveToPosePreAngle(SuperStructurePose pose) {
        return elevator.moveToPosition(pose.elevatorPosition)
            .alongWith(arm.moveToAngle(getMovingAngle(pose)))
            .andThen(arm.moveToAngle(pose.armAngle))
            .finallyDo(() -> { currentPose = pose; });
    }

    public Command retractArm(Angle retractAngle) {
        return arm.moveToAngle(retractAngle);
    }
}

