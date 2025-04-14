// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.subsystems.intake.IntakeConstants.IntakeAction;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {
    private final IntakeSimulation intakeSimulation;
    private final SwerveDriveSimulation driveSimulation;
    private final Elevator elevator;
    private final Arm arm;
    private AngularVelocity intakeSpeed = RotationsPerSecond.of(0);

    public IntakeIOSim(SwerveDriveSimulation driveSimulation, Elevator elevator, Arm arm) {
        this.driveSimulation = driveSimulation;
        this.elevator = elevator;
        this.arm = arm;
        this.intakeSimulation = IntakeSimulation.InTheFrameIntake(
                "Coral", driveSimulation, Inches.of(20), IntakeSimulation.IntakeSide.FRONT, 1);
    }

    public void addSimulatedGamePiece() {
        this.intakeSimulation.addGamePieceToIntake();
    }

    @Override
    public void setSpeed(AngularVelocity targetSpeed) {
        intakeSpeed = targetSpeed;
        if (targetSpeed.lt(RotationsPerSecond.of(0)) && intakeSimulation.obtainGamePieceFromIntake()) {
            Pose3d coralPose = new Pose3d();
            coralPose = coralPose.transformBy(new Transform3d(
                    Inches.of(11.625),
                    Inches.of(0),
                    Inches.of(7.8),
                    new Rotation3d(Degrees.of(0), Degrees.of(90), Degrees.of(0))));
            coralPose = coralPose.rotateBy(
                    new Rotation3d(Degrees.of(0), arm.getArmPosition().times(-1).plus(Degrees.of(13)), Degrees.of(0)));
            coralPose = new Pose3d(
                    coralPose.getMeasureX(),
                    coralPose.getMeasureY(),
                    coralPose.getMeasureZ().plus(Inches.of(21.875).plus(elevator.getPosition())),
                    coralPose.getRotation());
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                            driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                            arm.getArmPosition().gt(Degrees.of(0))
                                    ? new Translation2d(coralPose.getMeasureX().times(-1), coralPose.getMeasureY())
                                    : new Translation2d(coralPose.getMeasureX(), coralPose.getMeasureY()),
                            driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            arm.getArmPosition().gt(Degrees.of(0))
                                    ? driveSimulation
                                            .getSimulatedDriveTrainPose()
                                            .getRotation()
                                            .plus(Rotation2d.k180deg)
                                    : driveSimulation
                                            .getSimulatedDriveTrainPose()
                                            .getRotation(),
                            coralPose.getMeasureZ(),
                            // convert rotational velocity to linear
                            // v_linear = (2 * PI * r) * v_rotation
                            // 2*PI = circumference
                            // angular velocity = rate of change of angle
                            InchesPerSecond.of(
                                    Math.PI * 2.0 * targetSpeed.in(RotationsPerSecond) * 2), // 2PI * rps * 2?
                            coralPose.getRotation().getMeasureY().plus(Degrees.of(180))));
        }
    }

    @Override
    public void stop() {
        intakeSimulation.stopIntake();
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.hasCoral = intakeSimulation.getGamePiecesAmount() != 0;
        Logger.recordOutput("Intake/intakePieces", intakeSimulation.getGamePiecesAmount());

        if (intakeSpeed.isEquivalent(IntakeAction.INTAKING.speed) && !inputs.hasCoral) {
            Pose2d currentPose = driveSimulation.getSimulatedDriveTrainPose();
            Pose2d reletaveLoadingPose = currentPose.relativeTo(CoralStation.getClosestCoralStation(currentPose));
            Distance xOffset = reletaveLoadingPose.getMeasureX().minus(Meters.of(0.48));
            if (xOffset.lt(Inches.of(4))
                    && Math.abs(reletaveLoadingPose.getRotation().getDegrees()) - 180 >= -2.0) {
                addSimulatedGamePiece();
            }
            Logger.recordOutput("Loading/simXOffset", xOffset.in(Inches));
            Logger.recordOutput(
                    "Loading/simRotationOffset",
                    Math.abs(reletaveLoadingPose.getRotation().getDegrees()) - 180);
        }
    }
}
