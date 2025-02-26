package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Arrays;
import java.util.List;

public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);

    public static class Reef {
        public static final Translation2d center =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));

        private static final double PIPE_FROM_REEF_CENTER_INCHES =
                6.469; // taken from FieldConstants adjustY for reef y offset
        public static final Pose2d[] blueCenterFaces = new Pose2d[6];
        public static final Pose2d[] redCenterFaces = new Pose2d[6];

        public enum Side {
            LEFT,
            RIGHT
        };

        public static List<Pose2d> getAllianceReefList() {
            boolean isRed = DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red;
            return Arrays.asList(isRed ? redCenterFaces : blueCenterFaces);
        }

        public static Pose2d offsetReefPose(Pose2d facePose, Side side) {
            final double distanceFromReef = 0.48;
            final double rightOffset = PIPE_FROM_REEF_CENTER_INCHES - 2.7;
            final double leftOffset = PIPE_FROM_REEF_CENTER_INCHES + 2.7; // 2.4
            if (side == Side.LEFT) {
                return facePose.transformBy(new Transform2d(
                        distanceFromReef, // Robot length / 2
                        -Units.inchesToMeters(leftOffset),
                        Rotation2d.fromDegrees(0.0)));
            } else {
                return facePose.transformBy(new Transform2d(
                        distanceFromReef, // Robot length / 2
                        Units.inchesToMeters(rightOffset),
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
}
