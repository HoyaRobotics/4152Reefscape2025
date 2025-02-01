// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {
    @AutoLog
    public class ClimberInputs {
        public Pose3d pose;
        public Angle angle;
    }

    void updateInputs(ClimberInputs inputs);

    void setAngle(Angle targetAngle);
}
