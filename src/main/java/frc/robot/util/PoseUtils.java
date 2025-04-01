// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import java.util.function.Supplier;

/** Add your docs here. */
public class PoseUtils {
    public static Distance distanceBetweenPoses(Pose2d pose1, Pose2d pose2) {
        return Meters.of(pose1.getTranslation().getDistance(pose2.getTranslation()));
    }

    public static boolean poseInRange(Supplier<Pose2d> referencePose, Supplier<Pose2d> targetPose, Distance range) {
        return PoseUtils.distanceBetweenPoses(referencePose.get(), targetPose.get())
                .lt(range);
    }
}
