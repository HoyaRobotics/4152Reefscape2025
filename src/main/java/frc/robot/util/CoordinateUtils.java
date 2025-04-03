package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class CoordinateUtils {
    private static final Distance fieldWidth = Inches.of(317);
    private static final Distance fieldLength = Inches.of(690.876);

    public static boolean isRed() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
    }

    public static Pose2d getAlliancePose(Pose2d pose) {
        return new Pose2d(
                getAllianceX(pose.getMeasureX()),
                getAllianceY(pose.getMeasureY()),
                getAllianceRotation(pose.getRotation()));
    }

    public static Distance getAllianceY(Distance yDist) {
        return isRed() ? fieldWidth.minus(yDist) : yDist;
    }

    public static Distance getAllianceX(Distance xDist) {
        return isRed() ? fieldLength.minus(xDist) : xDist;
    }

    public static Rotation2d getAllianceRotation(Rotation2d rotation) {
        return isRed() ? rotation.plus(Rotation2d.k180deg) : rotation;
    }

    public static Distance clampAllianceDistance(Distance distance, Distance min, Distance max, boolean xDistance) {
        min = xDistance ? getAllianceX(min) : getAllianceY(min);
        max = xDistance ? getAllianceX(max) : getAllianceY(max);

        if (isRed()) {
            var temp = min;
            min = max;
            max = temp;
        }

        return Meters.of(Math.min(max.in(Meters), Math.max(distance.in(Meters), min.in(Meters))));
    }
}
