// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
    private static final double ANGLE_KP = 1.0 * TunerConstants.kDriveGearRatio;
    private static final double ANGLE_KD = 0.0; // 0.4
    private static final double ANGLE_MAX_VELOCITY = 10.0;
    private static final double ANGLE_MAX_ACCELERATION = 15.0;

    private final Angle angleDeltaTolerance;

    private final Drive drive;

    private final Supplier<Pose2d> poseSupplier;

    // 0.946 * gear ratio
    private final PIDController xController = new PIDController(0.946 * TunerConstants.kDriveGearRatio, 0.0, 0.0);
    private final PIDController yController = new PIDController(0.946 * TunerConstants.kDriveGearRatio, 0.0, 0.0);

    private final ProfiledPIDController angleController = new ProfiledPIDController(
            ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

    /** Creates a new DriveToPose. */
    public DriveToPose(Drive drive, Supplier<Pose2d> poseSupplier, Optional<Angle> angleDeltaTolerance) {
        addRequirements(drive);
        this.drive = drive;
        this.angleDeltaTolerance = angleDeltaTolerance.orElse(Degrees.of(5.0));
        this.poseSupplier = poseSupplier;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        angleController.enableContinuousInput(-Math.PI, Math.PI);

        Pose2d endPose = poseSupplier.get();

        angleController.reset(drive.getRotation().getRadians(), drive.getFieldChassisSpeeds().omegaRadiansPerSecond);
        xController.reset();
        yController.reset();

        xController.setSetpoint(endPose.getX());
        xController.setTolerance(Units.inchesToMeters(0.5));

        yController.setSetpoint(endPose.getY());
        yController.setTolerance(Units.inchesToMeters(0.5));

        angleController.setGoal(endPose.getRotation().getRadians());
        angleController.setTolerance(Units.degreesToRadians(2));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double angleSpeed = angleController.calculate(drive.getRotation().getRadians());
        // angleSpeed = MathUtil.clamp(angleSpeed, -3.8, 3.8);
        double xSpeed = 0.0;
        double ySpeed = 0.0;

        // starts driving once almost fully turned
        if (Math.abs(angleController.getPositionError()) < angleDeltaTolerance.in(Radians)) {
            xSpeed = xController.calculate(drive.getPose().getX());
            ySpeed = yController.calculate(drive.getPose().getY());
        }

        Logger.recordOutput("PIDToPose/xSpeed", xSpeed);
        Logger.recordOutput("PIDToPose/ySpeed", ySpeed);

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, angleSpeed);
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
    }

    // todo: eject coral button

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && angleController.atGoal();
    }
}
