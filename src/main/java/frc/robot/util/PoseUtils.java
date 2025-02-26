// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class PoseUtils {
    public static Distance distanceBetweenPoses(Pose2d pose1, Pose2d pose2) {
        return Meters.of(pose1.getTranslation().getDistance(pose2.getTranslation()));
    }
}
