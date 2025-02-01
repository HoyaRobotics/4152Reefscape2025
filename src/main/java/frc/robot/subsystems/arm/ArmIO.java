// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {
    @AutoLog
    class ArmInputs {
        public Pose3d pose;
        public Angle armAngle;
        public boolean hasGamePiece;
    }

    void updateInputs(ArmInputs inputs);

    void setArmPosition(Angle targetAngle);

    void setIntakeSPeed(double speed);

    void stopIntake();

    void stopArm();
}
