package frc.robot.constants;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Inches;

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
        public static final Pose2d LeftSidePose = new Pose2d(0.89, 7.32, Rotation2d.fromDegrees(-54));

        private static boolean inCoralStationRange = false;

        public static Pose2d getCoralStationPose(Side side) {
            boolean isRed = DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
            Pose2d coralStation = side == Side.RIGHT ? RightSidePose : LeftSidePose;
            if (isRed) {
                coralStation = new Pose2d(
                        FieldMirroringUtils.flip(coralStation.getTranslation()),
                        FieldMirroringUtils.flip(coralStation.getRotation()));
            }
            return coralStation.transformBy(
                    new Transform2d(0.36, Units.inchesToMeters(2.5), Rotation2d.fromDegrees(180))); // 0.48
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
        public static final Pose2d[] blueCenterFaces = new Pose2d[6];
        public static final Pose2d[] redCenterFaces = new Pose2d[6];

        public static AlgaeLevel getNearestAlgaePoses(Drive drive) {
            Pose2d centerFace = drive.getPose().nearest(getAllianceReefList());
            int faceIndex = getAllianceReefList().indexOf(centerFace);

            return faceIndex % 2 == 0 ? AlgaeLevel.ALGAE_L2 : AlgaeLevel.ALGAE_L3;
        }

        public static Pose2d getClosestBranchPose(Drive drive, Side side) {
            return offsetReefPose(drive.getPose().nearest(getAllianceReefList()), side);
        }

        public static List<Pose2d> getAllianceReefList() {
            boolean isRed = DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
            return Arrays.asList(isRed ? redCenterFaces : blueCenterFaces);
        }

        public static Pose2d offsetReefPose(Pose2d facePose, Side side) {
            final double distanceFromReef = 0.48;
            final double rightOffset = PIPE_FROM_REEF_CENTER_INCHES - 0.5; // 1.2 V1
            final double leftOffset = PIPE_FROM_REEF_CENTER_INCHES + 0.5; // 1.45 V1
            if (side == Side.LEFT) {
                return facePose.transformBy(new Transform2d(
                        distanceFromReef, // Robot length / 2
                        -Units.inchesToMeters(leftOffset),
                        Rotation2d.fromDegrees(0.0)));
            } else if (side == Side.RIGHT) {
                return facePose.transformBy(new Transform2d(
                        distanceFromReef, // Robot length / 2
                        Units.inchesToMeters(rightOffset),
                        Rotation2d.fromDegrees(0.0)));
            } else {
                return facePose.transformBy(new Transform2d(
                        distanceFromReef, // Robot length / 2
                        0.0,
                        Rotation2d.fromDegrees(0.0)));
            }
        }

        static {
            // Initialize faces
            blueCenterFaces[0] = new Pose2d(
                    Units.inchesToMeters(144.003), Units.inchesToMeters(158.500), Rotation2d.fromDegrees(180));
            blueCenterFaces[1] = new Pose2d(
                    Units.inchesToMeters(160.373), Units.inchesToMeters(186.857), Rotation2d.fromDegrees(120));
            blueCenterFaces[2] = new Pose2d(
                    Units.inchesToMeters(193.116), Units.inchesToMeters(186.858), Rotation2d.fromDegrees(60));
            blueCenterFaces[3] =
                    new Pose2d(Units.inchesToMeters(209.489), Units.inchesToMeters(158.502), Rotation2d.fromDegrees(0));
            blueCenterFaces[4] = new Pose2d(
                    Units.inchesToMeters(193.118), Units.inchesToMeters(130.145), Rotation2d.fromDegrees(-60));
            blueCenterFaces[5] = new Pose2d(
                    Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(-120));

            double allianceReefCenterDiff = fieldLength - (2 * Reef.center.getX());
            for (int i = 0; i < 6; ++i) {
                Pose2d blueFace = blueCenterFaces[i];
                redCenterFaces[i] =
                        new Pose2d(blueFace.getX() + allianceReefCenterDiff, blueFace.getY(), blueFace.getRotation());
            }
        }
    }

    public static class Net {
        // distance from robot bumper to center -> 0.48 meters?
        public static final Distance xOffset = Inches.of(303.5);
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
