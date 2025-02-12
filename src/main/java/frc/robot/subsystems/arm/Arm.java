// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

    private final ArmIO io;
    private ArmInputsAutoLogged inputs;
    private Elevator elevator;

    /** Creates a new Arm. */
    public Arm(ArmIO io, Elevator elevator) {
        this.io = io;
        this.inputs = new ArmInputsAutoLogged();
        this.elevator = elevator;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        this.io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        Pose3d armPose = new Pose3d(
                0.0,
                Units.inchesToMeters(6.625),
                Units.inchesToMeters(21.875) + elevator.getPosition(),
                new Rotation3d(Degrees.of(0), inputs.armAngle, Degrees.of(0)));
        Logger.recordOutput("Arm/ArmPose", armPose);
    }

    public void setIntakeSpeed(double speed) {

    }

    public void setArmPosition(Angle targetAngle) {
        this.io.setArmPosition(targetAngle);
    }

    boolean hasGamePiece() {
        return this.inputs.hasGamePiece;
    }

    boolean isArmAtPosition(Angle queriedAngle) {
        // TODO: implement
        return false;
    }

    Angle getArmPosition() {
        return this.inputs.armAngle;
    }

    void stopIntake() {}

    void stopArm() {}
}
