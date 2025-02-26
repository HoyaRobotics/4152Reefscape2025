package frc.robot.subsystems.superstructure;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import edu.wpi.first.units.measure.Angle;

public class SuperStructure {
    public enum SuperStructurePose {
        // constants call the constructor
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

    private final Elevator elevator;
    private final Arm arm;

    SuperStructure(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
    }

    public Command moveToPose(SuperStructurePose pose) {
        return elevator.moveToPosition(pose.elevatorPosition)
            .alongWith(arm.moveToAngle(pose.armAngle));
    }
}

