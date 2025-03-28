package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.SuperStructure.AlgaeLevel;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly.CoralStationsSide;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);

    public enum Side {
        LEFT,
        CENTER,
        RIGHT
    }

    public static class CoralStation {
        public static final Pose2d RightSidePose = new Pose2d(0.89, 0.6, Rotation2d.fromDegrees(54));
        public static final Pose2d LeftSidePose = new Pose2d(0.89, 7.43, Rotation2d.fromDegrees(-54)); // y: 7.32

        private static boolean inCoralStationRange = false;

        public static Pose2d getClosestCoralStation(Pose2d drivePose) {
            return drivePose.nearest(List.of(
                    FieldMirroringUtils.toCurrentAlliancePose(RightSidePose),
                    FieldMirroringUtils.toCurrentAlliancePose(LeftSidePose)));
        }

        public static Pose2d getCoralStationPose(Side side) {
            Pose2d coralStation = side == Side.RIGHT ? RightSidePose : LeftSidePose;
            coralStation = FieldMirroringUtils.toCurrentAlliancePose(coralStation);
            Logger.recordOutput("CoralStation/goal", coralStation);
            return coralStation.transformBy(
                    new Transform2d(0.4825, Units.inchesToMeters(2.5), Rotation2d.fromDegrees(180))); // 0.48
        }

        public static void simulateHumanPlayer(RobotContainer robotContainer) {
            boolean isRed = DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
            Pose2d rightSidePose = new Pose2d(0.89, 0.6, Rotation2d.fromDegrees(54));
            Angle angleDiff = robotContainer
                    .driveSimulation
                    .getSimulatedDriveTrainPose()
                    .getRotation()
                    .minus(Rotation2d.fromDegrees(54))
                    .getMeasure()
                    .plus(Degrees.of(180));
            Pose2d robotPose = robotContainer.driveSimulation.getSimulatedDriveTrainPose();
            Logger.recordOutput("CoralStation/RightSidePose", rightSidePose);
            if (isRed) {
                rightSidePose = new Pose2d(
                        FieldMirroringUtils.flip(rightSidePose.getTranslation()),
                        FieldMirroringUtils.flip(rightSidePose.getRotation()));
            }
            Logger.recordOutput("CoralStation/RightSidePoseAdjusted", rightSidePose);
            Transform2d poseDiff =
                    robotContainer.driveSimulation.getSimulatedDriveTrainPose().minus(rightSidePose);
            Distance difference =
                    Meters.of(Math.sqrt((poseDiff.getX() * poseDiff.getX()) + (poseDiff.getY() * poseDiff.getY())));

            boolean enteredRange = difference.lt(Meters.of(1.25));
            // && Math.abs(angleDiff.in(Degrees)) < 10.0;
            if (enteredRange && !inCoralStationRange) {
                inCoralStationRange = true;
                // right side station not field relative??
                SimulatedArena.getInstance()
                        .addGamePieceProjectile(ReefscapeCoralOnFly.DropFromCoralStation(
                                CoralStationsSide.RIGHT_STATION,
                                DriverStation.getAlliance().get(),
                                false));
            } else if (difference.gt(Meters.of(1.25))) {
                inCoralStationRange = false;
            }
        }
    }

    public static class Reef {

        public enum ReefLevel {
            TROUGH,
            L2,
            L3,
            L4;
        }

        public static final Translation2d center =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));

        private static final double PIPE_FROM_REEF_CENTER_INCHES =
                6.469; // taken from FieldConstants adjustY for reef y offset
        public static final Pose2d[] blueCenterFaces = {
            new Pose2d(Units.inchesToMeters(144.003), Units.inchesToMeters(158.500), Rotation2d.fromDegrees(180)),
            new Pose2d(Units.inchesToMeters(160.373), Units.inchesToMeters(186.857), Rotation2d.fromDegrees(120)),
            new Pose2d(
                    Units.inchesToMeters(192.913),
                    Units.inchesToMeters(186.858),
                    Rotation2d.fromDegrees(60)), // 193.116
            new Pose2d(Units.inchesToMeters(209.489), Units.inchesToMeters(158.502), Rotation2d.fromDegrees(0)),
            new Pose2d(
                    Units.inchesToMeters(192.913),
                    Units.inchesToMeters(130.145),
                    Rotation2d.fromDegrees(-60)), // 193.118
            new Pose2d(Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(-120))
        };

        public static final Pose2d[] redCenterFaces = Arrays.stream(blueCenterFaces)
                .map(blueFace -> new Pose2d(
                        FieldMirroringUtils.flip(blueFace.getTranslation()),
                        FieldMirroringUtils.flip(blueFace.getRotation())))
                .toArray(Pose2d[]::new);

        public static AlgaeLevel getNearestAlgaePoses(Drive drive) {
            Pose2d centerFace = drive.getPose().nearest(getAllReefLists());
            int faceIndex = getAllReefLists().indexOf(centerFace);

            return faceIndex % 2 == 1 ? AlgaeLevel.ALGAE_L2 : AlgaeLevel.ALGAE_L3;
        }

        public static Pose2d getAllianceReefBranch(int faceIndex, Side side) {
            return FieldMirroringUtils.toCurrentAlliancePose(offsetReefPose(blueCenterFaces[faceIndex], side));
        }

        public static Pose2d getClosestBranchPose(Drive drive, Side side) {
            return offsetReefPose(drive.getPose().nearest(getAllReefLists()), side);
        }

        public static List<Pose2d> getAllReefLists() {
            return Stream.concat(Arrays.stream(blueCenterFaces), Arrays.stream(redCenterFaces))
                    .toList();
        }

        public static List<Pose2d> getAllianceReefList() {
            boolean isRed = DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
            return Arrays.asList(isRed ? redCenterFaces : blueCenterFaces);
        }

        public static Pose2d offsetReefPose(Pose2d facePose, Side side) {
            final double distanceFromReef = 0.48;
            final double rightOffset = PIPE_FROM_REEF_CENTER_INCHES + 0.75; // 1.2 V1, - 0.5
            final double leftOffset = PIPE_FROM_REEF_CENTER_INCHES; // 1.45 V1

            final double yOffset = side == Side.CENTER
                    ? 0.0
                    : side == Side.RIGHT ? Units.inchesToMeters(rightOffset) : -Units.inchesToMeters(leftOffset);

            return facePose.transformBy(new Transform2d(distanceFromReef, yOffset, Rotation2d.fromDegrees(0.0)));
        }
    }

    public static class Processor {
        public static final Pose2d processorFaceBlue =
                new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
        public static final Pose2d processorFaceRed = new Pose2d(
                FieldMirroringUtils.flip(processorFaceBlue.getTranslation()),
                FieldMirroringUtils.flip(processorFaceBlue.getRotation()));

        public static Pose2d getProcessorPose() {
            boolean isRed = DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;

            Pose2d centerFace = isRed ? processorFaceRed : processorFaceBlue;
            return centerFace.transformBy(
                    new Transform2d(0.48 + Units.inchesToMeters(3), 0.0, Rotation2d.fromDegrees(180)));
        }
    }

    public static class Net {
        // distance from robot bumper to center -> 0.48 meters?
        public static final Distance xOffset = Inches.of(303.5 - 13); // 303.5
        // distance from arena wall to net
        public static final Distance yOffset = Inches.of(6.243 - 0.83);
        public static final Distance netWidth = Inches.of(148.130);
        public static final Rotation2d rotationOffset = Rotation2d.fromDegrees(0);

        public static Pose2d getNetPose(Pose2d drivePose) {
            boolean isRed = DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;

            Distance netEnd = Meters.of(fieldWidth).minus(yOffset.plus(netWidth).minus(Meters.of(0.48)));
            netEnd = isRed
                    ? FieldMirroringUtils.flip(new Translation2d(Meters.of(0), netEnd))
                            .getMeasureY()
                    : netEnd;

            Distance absoluteXOffset = isRed ? Meters.of(fieldLength).minus(xOffset) : xOffset;

            // clamp pose
            Distance yDistance = isRed
                    ? Meters.of(
                            Math.min(netEnd.in(Meters), drivePose.getMeasureY().in(Meters)))
                    : Meters.of(
                            Math.max(netEnd.in(Meters), drivePose.getMeasureY().in(Meters)));

            return new Pose2d(
                    new Translation2d(absoluteXOffset, yDistance),
                    isRed ? FieldMirroringUtils.flip(rotationOffset) : rotationOffset);
        }
    }
}
