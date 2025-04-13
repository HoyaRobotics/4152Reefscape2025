// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.constants.FieldConstants.StagingPositions;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeIntakeAction;
import java.util.List;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/** Add your docs here. */
public class AlgaeIntakeIOSim implements AlgaeIntakeIO {
    private final SwerveDriveSimulation driveSimulation;
    private AngularVelocity intakeSpeed = RotationsPerSecond.of(0);
    private boolean hasAlgae = false;

    public AlgaeIntakeIOSim(SwerveDriveSimulation driveSimulation) {
        this.driveSimulation = driveSimulation;
    }

    @Override
    public void setSpeed(AngularVelocity targetSpeed) {
        intakeSpeed = targetSpeed;

        // check for outtaking, intaking handled in update inputs
        if (targetSpeed.lt(RotationsPerSecond.of(0)) && hasAlgae) {
            // simulate projectile motion
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
