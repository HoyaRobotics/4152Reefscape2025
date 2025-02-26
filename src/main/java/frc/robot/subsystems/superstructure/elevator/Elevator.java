// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// McT testing Git
package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private ElevatorInputsAutoLogged inputs;
    private Pose3d stage2Pose;
    private Pose3d carriagePose;

    /** Creates a new Elevator. */
    public Elevator(ElevatorIO io) {
        this.io = io;
        this.inputs = new ElevatorInputsAutoLogged();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        // This method will be called once per scheduler run
        if (inputs.position.in(Meters) > Units.inchesToMeters(24.25)) {
            this.stage2Pose = new Pose3d(
                    0.0,
                    Units.inchesToMeters(-11.0),
                    inputs.position.in(Meters) + Units.inchesToMeters(4.875) - Units.inchesToMeters(24.25),
                    new Rotation3d());
        } else {
            this.stage2Pose =
                    new Pose3d(0.0, Units.inchesToMeters(-11.0), Units.inchesToMeters(4.875), new Rotation3d());
        }
        this.carriagePose = new Pose3d(
                0.0,
                Units.inchesToMeters(-11.0),
                inputs.position.in(Meters) + Units.inchesToMeters(5.875),
                new Rotation3d());
        Logger.recordOutput("Elevator/CarriagePose", this.carriagePose);
        Logger.recordOutput("Elevator/ElevatorPose", this.stage2Pose);
    }

    public Command moveToPosition(Distance targetPosition) {
        return Commands.run(() -> {}, this)
                .beforeStarting(() -> setPosition(targetPosition))
                .until(() -> isAtPosition(targetPosition));
    }

    public Pose3d getStage2Pose() {
        return this.stage2Pose;
    }

    public Pose3d getCarriagePose() {
        return this.carriagePose;
    }

    public void stop() {
        this.io.stop();
    }

    public boolean isAtPosition(Distance queriedPosition) {
        return queriedPosition.minus(inputs.position).abs(Inches) <= ElevatorConstants.positionError.in(Inches);
    }

    public Distance getPosition() {
        return this.inputs.position;
    }

    public void setPosition(Distance targetPosition) {
        this.io.setPosition(targetPosition);
    }
}
