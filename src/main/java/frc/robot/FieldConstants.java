package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static class Reef {
        public static final Translation2d center =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));

        private static final double PIPE_FROM_REEF_CENTER_INCHES =
                6.469; // taken from FieldConstants adjustY for reef y offset
        public static final Pose2d[] centerFaces = new Pose2d[6];

        public enum Side {
            LEFT,
            RIGHT
        };

        public static Pose2d offsetReefPose(Pose2d facePose, Side side) {
            if (side == Side.LEFT) {
                return facePose.transformBy(new Transform2d(
                        1.0, // Robot length / 2
                        -Units.inchesToMeters(PIPE_FROM_REEF_CENTER_INCHES),
                        Rotation2d.fromDegrees(180)));
            } else {
                return facePose.transformBy(new Transform2d(
                        1.0, // Robot length / 2
                        Units.inchesToMeters(PIPE_FROM_REEF_CENTER_INCHES),
                        Rotation2d.fromDegrees(180)));
            }
        }

        static {
            // Initialize faces
            centerFaces[0] = new Pose2d(
                    Units.inchesToMeters(144.003), Units.inchesToMeters(158.500), Rotation2d.fromDegrees(180));
            centerFaces[1] = new Pose2d(
                    Units.inchesToMeters(160.373), Units.inchesToMeters(186.857), Rotation2d.fromDegrees(120));
            centerFaces[2] = new Pose2d(
                    Units.inchesToMeters(193.116), Units.inchesToMeters(186.858), Rotation2d.fromDegrees(60));
            centerFaces[3] =
                    new Pose2d(Units.inchesToMeters(209.489), Units.inchesToMeters(158.502), Rotation2d.fromDegrees(0));
            centerFaces[4] = new Pose2d(
                    Units.inchesToMeters(193.118), Units.inchesToMeters(130.145), Rotation2d.fromDegrees(-60));
            centerFaces[5] = new Pose2d(
                    Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(-120));
        }
    }
}
