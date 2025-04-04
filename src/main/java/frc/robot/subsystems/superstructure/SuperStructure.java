package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.vision.Vision;
import java.util.Collections;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

// would be nice to alway know what level we are at?
public class SuperStructure {

    public static final Distance intakeSlopeRate =
            Inches.of(1.5); // distance to go down per 4.5in of coral infront of loading station

    public enum AlgaeLevel {
        ALGAE_L2,
        ALGAE_L3
    }
    // add deadline for intaking so it automatically gpoes to hold for austin

    // fix elevaot zeroing, move arm
    public enum SuperStructurePose {
        // constants call the constructor

        // Elevator position, arm angle
        TROUGH(Inches.of(7.5), Degrees.of(-60)),
        POST_TROUGH(Inches.of(12), Degrees.of(-60)),
        POST_POST_TROUGH(Inches.of(16.75), Degrees.of(-60)),
        STAGED_ALGAE(Inches.of(0.0), Degrees.of(5)),
        // TROUGH(Inches.of(0), Degrees.of(135)),
        // POST_TROUGH(Inches.of(0), Degrees.of(110)),
        // TROUGH(Inches.of(0), Degrees.of(120)),
        LOADING(Inches.of(16.75), Degrees.of(-35)), // 18
        MAX_LOADING(Inches.of(17.85), Degrees.of(-35)),
        MIN_LOADING(Inches.of(0.0), Degrees.of(-20)),
        LOADING_CORAL_BETW(Inches.of(17), Degrees.of(-35)),
        PROCESSOR(Inches.of(0.0), Degrees.of(0)),
        L2(Inches.of(11.5), Degrees.of(135.0)), // 13.0
        L3(Inches.of(27.25), Degrees.of(135.0)), // 15.75 more than L2
        L4(Inches.of(52.7), Degrees.of(146.0)), // 143

        L2_ALGAE(Inches.of(4.0), Degrees.of(160)),
        L2_ALGAE_GRAB(Inches.of(7.25), Degree.of(160)),
        L2_ALGAE_REMOVE(Inches.of(0.0), Degree.of(103)),
        L3_ALGAE(Inches.of(19.75), Degrees.of(160)), // 15.75 more than L2 20.75
        L3_ALGAE_GRAB(Inches.of(23.0), Degrees.of(160)), // 15.75 more than L2 25.75
        L3_ALGAE_REMOVE(Inches.of(9.25), Degrees.of(103)), // 15.75 more than L2 10.75

        // ALGAE_PRE_NET(Inches.of(52.7), Degrees.of(103)),
        ALGAE_NET(Inches.of(52.7), Degrees.of(83)),
        ZERO(Inches.of(0.0), Degrees.of(103)),
        HOLD(Inches.of(3.0), Degrees.of(103)),
        CLIMB_STOW(Inches.of(0.0), Degrees.of(-15)),
        BASE(Inches.of(16.75), Degrees.of(-35)); // 3.0, -25

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
        switch (pose) {
            case L2:
            case L3:
            case L4:
            case ALGAE_NET:
                return Degrees.of(103);
            default:
                return pose.armAngle;
        }
    }

    public Command moveToPose(SuperStructurePose pose) {
        return elevator.moveToPosition(pose.elevatorPosition, true)
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

    public Command moveToLoadingPose(Drive drive, Vision vision) {
        // LinearFilter filter = LinearFilter.movingAverage(1);
        return Commands.run(
                        () -> {
                            Pose2d currentPose = drive.getPose();
                            Pose2d targetPose = CoralStation.getClosestCoralStation(currentPose);

                            final Distance minHeight = SuperStructurePose.MIN_LOADING.elevatorPosition;
                            final Distance maxHeight = SuperStructurePose.MAX_LOADING.elevatorPosition;
                            final Angle minAngle = SuperStructurePose.MIN_LOADING.armAngle;
                            final Angle maxAngle = SuperStructurePose.MAX_LOADING.armAngle;

                            final double predictionGain = 0.12;

                            var targetRelVelocity = Math.max(
                                    0.0,
                                    DriveCommands.getTargetRelativeLinearVelocity(drive, targetPose)
                                            .getX());

                            double xOffset = currentPose
                                    .relativeTo(CoralStation.getClosestCoralStation(currentPose))
                                    .getMeasureX()
                                    .minus(Meters.of(0.48))
                                    .minus(Meters.of(Math.abs(targetRelVelocity) * predictionGain))
                                    .abs(Inches);

                            if (xOffset < Units.metersToInches(0.5)) vision.disableCamera(0);

                            // xOffset = filter.calculate(xOffset);

                            // start using filter once within 1 corals range??

                            Logger.recordOutput("Loading/targetRelVelocity", targetRelVelocity);
                            Logger.recordOutput("Loading/xOffset", xOffset);

                            Distance height = maxHeight.minus(Inches.of(xOffset * 4.5 / 4.5));
                            Distance inputHeight = height.gt(minHeight) ? height : minHeight;

                            Angle angle = maxAngle.plus(Degrees.of(xOffset * 2.5 / 4.5));
                            Angle inputAngle = angle.lt(minAngle) ? angle : minAngle;

                            elevator.setPosition(inputHeight, false);
                            arm.setArmPosition(inputAngle);
                        },
                        arm,
                        elevator)
                .finallyDo(() -> vision.enableCamera(0));
    }

    public boolean isAtPosition() {
        return elevator.isAtPosition(targetPose.elevatorPosition) && arm.isArmAtPosition(targetPose.armAngle);
    }

    public BooleanSupplier waitTillRetracted() {
        return () -> arm.getArmPosition().minus(ArmConstants.safeRetractAngle).lt(Degrees.of(0));
    }
}
