// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// McT testing Git
package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

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

    /*
    public Command moveToLoadingPose(Supplier<Pose2d> drivePose) {
        // LinearFilter filter = LinearFilter.movingAverage(1);
        return Commands.run(
                () -> {
                    Pose2d currentPose = drivePose.get();
                    final Distance minHeight = SuperStructurePose.MIN_LOADING.elevatorPosition;
                    final Distance maxHeight = SuperStructurePose.MAX_LOADING.elevatorPosition;
                    Distance xOffset = currentPose
                            .relativeTo(CoralStation.getClosestCoralStation(currentPose))
                            .getMeasureX()
                            .minus(Meters.of(0.48));
                    Logger.recordOutput("Loading/xOffset", xOffset.in(Inches));

                    Distance height = maxHeight.minus(Inches.of(xOffset.in(Inches) * 3.0 / 4.5));
                    Distance inputHeight = height.gt(minHeight) ? height : minHeight;
                    inputHeight = inputHeight.lt(maxHeight) ? inputHeight : maxHeight;
                    Logger.recordOutput("Loading/inputHeight", inputHeight.in(Inches));
                    // setPosition(Inches.of(filter.calculate(inputHeight.in(Inches))), false);
                    setPosition(inputHeight, false);
                },
                this);
    }*/

    public Command moveToPosition(Distance targetPosition, boolean motionMagic) {
        return Commands.run(() -> {}, this)
                .beforeStarting(() -> setPosition(targetPosition, motionMagic))
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

    public void setPosition(Distance targetPosition, boolean motionMagic) {
        this.io.setPosition(targetPosition, motionMagic);
    }

    public Command zeroPosition() {
        return Commands.run(() -> this.io.setVoltage(Volts.of(-2.0)), this)
                .beforeStarting(() -> this.io.changeSoftLimits(false))
                .until(() -> this.io.getCurrent().gt(Amps.of(35.0)))
                .finallyDo(() -> {
                    this.io.zeroEncoder();
                    this.io.changeSoftLimits(true);
                });
    }
}
