// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveToPose;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoseProfiled extends Command {
    private static double driveKp = 0.43 * TunerConstants.kDriveGearRatio;
    private static double driveKd = 0.0;

    private static double driveMaxVelocity = 4.73;
    private static final double driveMaxAcceleration = 11.772; // 7.89

    private static final double thetaKp = 1.0 * TunerConstants.kDriveGearRatio;
    private static final double thetaKd = 0.0; // 0.4
    private static final double thetaMaxVelocity = 10.0;
    private static final double thetaMaxAcceleration = 15.0;

    private static final double linearFFGain = 3.0;

    private Translation2d lastSetpointTranslation = Translation2d.kZero;

    private final Drive drive;

    private final Supplier<Pose2d> poseSupplier;
    private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;

    private final ProfiledPIDController driveController = new ProfiledPIDController(
            driveKp, 0.0, driveKd, new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));

    private final ProfiledPIDController angleController = new ProfiledPIDController(
            thetaKp, 0.0, thetaKd, new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));

    /** Creates a new DriveToPose. */
    public DriveToPoseProfiled(Drive drive, Supplier<Pose2d> poseSupplier) {
        addRequirements(drive);
        this.drive = drive;
        this.poseSupplier = poseSupplier;
    }

    public DriveToPoseProfiled(Drive drive, Pose2d pose) {
        addRequirements(drive);
        this.drive = drive;
        this.poseSupplier = () -> pose;
    }

    public DriveToPoseProfiled(Drive drive, Supplier<Pose2d> poseSupplier, Supplier<Translation2d> linearFF) {
        addRequirements(drive);
        this.drive = drive;
        this.poseSupplier = poseSupplier;
        this.linearFF = linearFF;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        angleController.enableContinuousInput(-Math.PI, Math.PI);

        Pose2d currentPose = drive.getPose();
        Pose2d targetPose = poseSupplier.get();
        // this is not robot relative?

        // velocity should represent the rate of change of the distance between robot and target,
        // which would decrease over time
        // double diffDelta = Math.min(0.0, -fieldVelocity.dot(targetRelative));
        double diffDelta = Math.min(
                0.0,
                -DriveCommands.getTargetRelativeLinearVelocity(drive, targetPose)
                        .getX());

        Logger.recordOutput("DriveToPose/diffDelta", diffDelta);

        angleController.reset(drive.getRotation().getRadians(), drive.getFieldChassisSpeeds().omegaRadiansPerSecond);
        driveController.reset(currentPose.getTranslation().getDistance(targetPose.getTranslation()), diffDelta);

        driveController.setTolerance(Units.inchesToMeters(0.5));
        angleController.setTolerance(Units.degreesToRadians(2));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();
        Pose2d targetPose = poseSupplier.get();

        driveController.reset(
                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity);

        double thetaVelocity = angleController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        double driveVelocityScalar =
                driveController.calculate(currentPose.getTranslation().getDistance(targetPose.getTranslation()), 0.0);

        // create a pose of length zero pointing towards the target from the robot
        // move along that line by the drive velocity sclara by transforming by X
        // remember x -> outwards relative towards the target
        var driveVelocity = new Pose2d(
                        Translation2d.kZero,
                        new Rotation2d(Math.atan2(
                                currentPose.getTranslation().getY()
                                        - targetPose.getTranslation().getY(),
                                currentPose.getTranslation().getX()
                                        - targetPose.getTranslation().getX())))
                .transformBy(new Transform2d(driveVelocityScalar, 0.0, Rotation2d.kZero))
                .getTranslation();

        if (Constants.FuseDriverInputs) {
                var linearT = linearFF.get().getNorm() * linearFFGain;
                driveVelocity = driveVelocity.interpolate(linearFF.get().times(driveMaxVelocity), linearT);
        }

        lastSetpointTranslation = new Pose2d(
                        targetPose.getTranslation(),
                        new Rotation2d(Math.atan2(
                                currentPose.getTranslation().getY()
                                        - targetPose.getTranslation().getY(),
                                currentPose.getTranslation().getX()
                                        - targetPose.getTranslation().getX())))
                .transformBy(new Transform2d(driveController.getSetpoint().position, 0.0, Rotation2d.kZero))
                .getTranslation();

        Logger.recordOutput("DriveToPose/targetPose", targetPose);
        Logger.recordOutput("DriveToPose/driveAtGoal", driveController.atGoal());
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
        return driveController.atGoal() && angleController.atGoal();
        // return false;
    }
}
