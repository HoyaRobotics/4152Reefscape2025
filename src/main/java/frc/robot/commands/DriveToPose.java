// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
    private static double driveKp = 0.43 * TunerConstants.kDriveGearRatio;
    private static double driveKd = 0.1;

    private static double driveMaxVelocity = 4.73;
    private static final double driveMaxAcceleration = 11.772; // 7.89

    private static final double thetaKp = 1.0 * TunerConstants.kDriveGearRatio;
    private static final double thetaKd = 0.0; // 0.4
    private static final double thetaMaxVelocity = 10.0;
    private static final double thetaMaxAcceleration = 15.0;

    private final Drive drive;

    private final Supplier<Pose2d> targetPose;
    private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;

    private final PIDController driveController = new PIDController(driveKp, 0.0, driveKd);

    private final ProfiledPIDController angleController = new ProfiledPIDController(
            thetaKp, 0.0, thetaKd, new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));

    /** Creates a new DriveToPose. */
    public DriveToPose(Drive drive, Supplier<Pose2d> targetPose) {
        addRequirements(drive);
        this.drive = drive;
        this.targetPose = targetPose;
    }

    public DriveToPose(Drive drive, Supplier<Pose2d> targetPose, Supplier<Translation2d> linearFF) {
        this.drive = drive;
        this.targetPose = targetPose;
        this.linearFF = linearFF;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        angleController.enableContinuousInput(-Math.PI, Math.PI);

        angleController.reset(drive.getRotation().getRadians(), drive.getFieldChassisSpeeds().omegaRadiansPerSecond);
        driveController.reset();
        driveController.setSetpoint(0);

        driveController.setTolerance(Units.inchesToMeters(0.5));
        angleController.setTolerance(Units.degreesToRadians(2));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();
        Pose2d target = targetPose.get();

        Logger.recordOutput("DriveToPose/targetPose", target);

        Logger.recordOutput(
                "DriveToPose/targetDistance", currentPose.getTranslation().getDistance(target.getTranslation()));

        double thetaVelocity = angleController.calculate(
                currentPose.getRotation().getRadians(), target.getRotation().getRadians());
        double driveVelocityScalar =
                driveController.calculate(currentPose.getTranslation().getDistance(target.getTranslation()));

        // remember x -> outwards relative towards the target
        var driveVelocity = new Pose2d(
                        Translation2d.kZero,
                        new Rotation2d(Math.atan2(
                                currentPose.getTranslation().getY()
                                        - target.getTranslation().getY(),
                                currentPose.getTranslation().getX()
                                        - target.getTranslation().getX())))
                .transformBy(new Transform2d(driveVelocityScalar, 0.0, Rotation2d.kZero))
                .getTranslation();

        // interpolate drive velocity towards joystick direction
        // by feed forward magnitude?
        final double linearScale = linearFF.get().getNorm() * 3.0;
        driveVelocity = driveVelocity.interpolate(linearFF.get().times(driveMaxVelocity), linearScale);

        Logger.recordOutput("DriveToPose/driveAtGoal", driveController.atSetpoint());
        Logger.recordOutput("DriveToPose/angleAtGoal", angleController.atGoal());
        // Convert to field relative speeds & send command
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, drive.getRotation()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return driveController.atSetpoint() && angleController.atGoal();
        // return false;
    }
}
