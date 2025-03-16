// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
    private static final double LINEAR_KP = 1.0 * TunerConstants.kDriveGearRatio;
    private static final double LINEAR_KI = 0.0;
    private static final double LINEAR_KD = 0.0;

    private static final double LINEAR_MAX_VELOCITY = 4.5;
    private static final double LINEAR_MAX_ACCELERATION = 5.7;

    private static final double ANGLE_KP = 1.0 * TunerConstants.kDriveGearRatio;
    private static final double ANGLE_KD = 0.0; // 0.4
    private static final double ANGLE_MAX_VELOCITY = 10.0;
    private static final double ANGLE_MAX_ACCELERATION = 15.0;

    private boolean withingAngleTolerance = false;

    private final Angle angleDeltaTolerance;

    private final Pair<Distance, Angle> controllerTolerance;

    private final Drive drive;

    private final Supplier<Pose2d> poseSupplier;

    private final ProfiledPIDController xController = new ProfiledPIDController(
            LINEAR_KP,
            LINEAR_KI,
            LINEAR_KD,
            new TrapezoidProfile.Constraints(LINEAR_MAX_VELOCITY, LINEAR_MAX_ACCELERATION));

    private final ProfiledPIDController yController = new ProfiledPIDController(
            LINEAR_KP,
            LINEAR_KI,
            LINEAR_KD,
            new TrapezoidProfile.Constraints(LINEAR_MAX_VELOCITY, LINEAR_MAX_ACCELERATION));

    private final ProfiledPIDController angleController = new ProfiledPIDController(
            ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

    private boolean stopDrive;

    /** Creates a new DriveToPose. */
    public DriveToPose(
            Drive drive,
            Supplier<Pose2d> poseSupplier,
            Optional<Angle> angleDeltaTolerance,
            Optional<Pair<Distance, Angle>> controllerTolerance,
            boolean limitSlewRate,
            boolean stopDrive) {
        addRequirements(drive);
        this.stopDrive = stopDrive;
        this.drive = drive;
        this.angleDeltaTolerance = angleDeltaTolerance.orElse(Degrees.of(5.0));
        this.poseSupplier = poseSupplier;
        this.controllerTolerance = controllerTolerance.orElse(new Pair<>(Inches.of(0.5), Degrees.of(2)));
        // this.linearXSlewFilter = limitSlewRate ? Optional.of(new SlewRateLimiter(4)) : Optional.empty();
        // this.linearYSlewFilter = limitSlewRate ? Optional.of(new SlewRateLimiter(4)) : Optional.empty();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Math.abs(angleController.getPositionError()) < angleDeltaTolerance.in(Radians))
            withingAngleTolerance = true;
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        Pose2d endPose = poseSupplier.get();

        angleController.reset(drive.getRotation().getRadians(), drive.getFieldChassisSpeeds().omegaRadiansPerSecond);
        xController.reset(drive.getPose().getX(), drive.getFieldChassisSpeeds().vxMetersPerSecond);
        yController.reset(drive.getPose().getY(), drive.getFieldChassisSpeeds().vyMetersPerSecond);

        xController.setGoal(endPose.getX());
        xController.setTolerance(controllerTolerance.getFirst().in(Meters));

        yController.setGoal(endPose.getY());
        yController.setTolerance(controllerTolerance.getFirst().in(Meters));

        angleController.setGoal(endPose.getRotation().getRadians());
        angleController.setTolerance(controllerTolerance.getSecond().in(Radians));
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
            if (!withingAngleTolerance) {
                xController.reset(drive.getPose().getX(), drive.getFieldChassisSpeeds().vxMetersPerSecond);
                yController.reset(drive.getPose().getY(), drive.getFieldChassisSpeeds().vyMetersPerSecond);

                xController.setGoal(poseSupplier.get().getX());
                yController.setGoal(poseSupplier.get().getY());
            }
            withingAngleTolerance = true;
            xSpeed = xController.calculate(drive.getPose().getX());
            ySpeed = yController.calculate(drive.getPose().getY());
        }

        // Convert to field relative speeds & send command
        Logger.recordOutput("PIDToPose/xSpeed", xSpeed);
        Logger.recordOutput("PIDToPose/ySpeed", ySpeed);
        /*
        if (linearXSlewFilter.isPresent()) {
            Logger.recordOutput("SlewRate/RawXspeed", xSpeed);
            Logger.recordOutput("SlewRate/RawYspeed", ySpeed);
            xSpeed = linearXSlewFilter.get().calculate(xSpeed);
            ySpeed = linearYSlewFilter.get().calculate(ySpeed);
            Logger.recordOutput("SlewRate/FilteredXspeed", xSpeed);
            Logger.recordOutput("SlewRate/FilteredYspeed", ySpeed);
        }
        */

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, angleSpeed);
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (stopDrive) drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && angleController.atGoal();
    }
}
