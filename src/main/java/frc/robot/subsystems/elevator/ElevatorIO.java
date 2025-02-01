// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorInputs {
        public Pose3d stage2Pose;
        public Pose3d stage3Pose;
        public double position;
    }

    void setPosition(double targetPosition);

    void stop();

    void updateInputs(ElevatorInputs inputs);

    void configureMotors();
}
