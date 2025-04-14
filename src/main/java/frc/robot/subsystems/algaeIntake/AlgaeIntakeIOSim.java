// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.constants.FieldConstants.StagingPositions;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeIntakeAction;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AlgaeIntakeIOSim implements AlgaeIntakeIO {
    private final SwerveDriveSimulation driveSimulation;
    private final Elevator elevator;
    private final Arm arm;
    private AngularVelocity intakeSpeed = RotationsPerSecond.of(0);
    private boolean hasAlgae = false;

    public AlgaeIntakeIOSim(SwerveDriveSimulation driveSimulation, Elevator elevator, Arm arm) {
        this.driveSimulation = driveSimulation;
        this.elevator = elevator;
        this.arm = arm;
    }

    @Override
    public void setSpeed(AngularVelocity targetSpeed) {
        intakeSpeed = targetSpeed;

        // check for outtaking, intaking handled in update inputs
        if (targetSpeed.lt(RotationsPerSecond.of(0)) && hasAlgae) {
            // simulate projectile motion
            Pose3d algaePose = new Pose3d();
            Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
            // algae position relative to arm pivot point
            algaePose = algaePose.transformBy(new Transform3d(
                    Inches.of(24.0), // outwards from elevator
                    Inches.of(0),
                    Inches.of(0),
                    Rotation3d.kZero));

            // roll, pitch, yaw
            algaePose = algaePose.rotateBy(new Rotation3d(
                    Degrees.of(0),
                    arm.getArmPosition().times(-1).plus(Degrees.of(13)),
                    robotPose.getRotation().getMeasure()));
            algaePose = new Pose3d(
                    algaePose.getMeasureX().plus(robotPose.getMeasureX()),
                    algaePose.getMeasureY().plus(robotPose.getMeasureY()),
                    algaePose.getMeasureZ().plus(Inches.of(21.875)).plus(elevator.getPosition()),
                    Rotation3d.kZero);

            Pose3d pivotRelPosition =
                    new Pose3d(Inches.of(-9.261), Inches.of(15.785), Inches.of(-0.048), Rotation3d.kZero);
            Distance intakeWheelRadius = Inches.of(2.0);

            Translation2d tangentialVelocity = new Translation2d(
                    arm.getRotationalVelocity().in(RadiansPerSecond)
                            * pivotRelPosition.getTranslation().getNorm(),
                    Rotation2d.fromDegrees(algaePose
                            .getRotation()
                            .getMeasureY()
                            .minus(Degrees.of(90))
                            .in(Degrees)));

            Translation2d intakeVelocity = new Translation2d(
                    intakeSpeed.in(RotationsPerSecond) * Math.PI * 2.0 * intakeWheelRadius.in(Meters),
                    Rotation2d.fromDegrees(algaePose.getRotation().getMeasureY().in(Degrees)));

            Translation2d elevatorVelocity =
                    new Translation2d(elevator.getVelocity().in(MetersPerSecond), Rotation2d.fromDegrees(90));

            LinearVelocity finalSpeed = MetersPerSecond.of(tangentialVelocity
                    .plus(intakeVelocity)
                    .plus(elevatorVelocity)
                    .getNorm());

            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                                    robotPose.getTranslation(),
                                    new Translation2d(algaePose.getMeasureX().times(-1), algaePose.getMeasureY()),
                                    driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                    robotPose.getRotation().plus(Rotation2d.k180deg),
                                    algaePose.getMeasureZ(),
                                    finalSpeed,
                                    algaePose.getRotation().getMeasureY()) // pitch
                            .withProjectileTrajectoryDisplayCallBack(
                                    (poses) -> Logger.recordOutput(
                                            "successfulShotsTrajectory", poses.toArray(Pose3d[]::new)),
                                    (poses) -> Logger.recordOutput(
                                            "missedShotsTrajectory", poses.toArray(Pose3d[]::new))));
            // just processor to start
            hasAlgae = false;
        }
    }

    @Override
    public void stop() {}

    @Override
    public void updateInputs(AlgaeIntakeInputs inputs) {
        if (intakeSpeed.isEquivalent(AlgaeIntakeAction.INTAKING.speed) && !hasAlgae) {
            Pose2d currentPose = driveSimulation.getSimulatedDriveTrainPose();

            Pose2d closestFace = Reef.getClosestBranchPose(() -> currentPose, Side.CENTER);
            Pose2d closestLollipop = currentPose.nearest(StagingPositions.getAllianceStartingAlgaePoses());

            Pose2d closestAlgae = currentPose.nearest(List.of(closestFace, closestLollipop));
            var xOffset = currentPose.relativeTo(closestAlgae).getMeasureX();
            var rotationError = currentPose.relativeTo(closestAlgae).getRotation();

            if (xOffset.lt(Inches.of(5.0)) && rotationError.getMeasure().lt(Degrees.of(8))) {
                hasAlgae = true;
            }
        }
        inputs.hasAlgae = hasAlgae;
    }
}
