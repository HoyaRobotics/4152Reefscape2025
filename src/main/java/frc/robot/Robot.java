// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly.CoralStationsSide;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    private boolean inCoralStationRange = false;

    public Robot() {
        CanBridge.runTCP();
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Start AdvantageKit logger
        Logger.start();

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Switch thread to high priority to improve loop timing
        Threads.setCurrentThreadPriority(true, 99);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 10);
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        robotContainer.resetSimulationField();
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // Pose3d robotPose = new Pose3d(robotContainer.drive.getPose());
        // Pose3d cameraPose = robotPose.transformBy(VisionConstants.robotToCamera0);
        // Logger.recordOutput("Vision/cameraPose", cameraPose);
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();

        // coral station logic
        // check if we are in a certain distance, drop once and wait until
        // we leave and come back within that distance to drop again
        // also wait for the robot to be facing it
        // difference between angles less than 20
        // make sides field relative, doesnt work on red
        boolean isRed = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        Pose2d rightSidePose = new Pose2d(0.89, 0.6, Rotation2d.fromDegrees(54));
        Angle angleDiff = robotContainer
                .driveSimulation
                .getSimulatedDriveTrainPose()
                .getRotation()
                .minus(Rotation2d.fromDegrees(54))
                .getMeasure()
                .plus(Degrees.of(180));
        Pose2d robotPose = robotContainer.driveSimulation.getSimulatedDriveTrainPose();
        Logger.recordOutput("CoralStation/RightSidePose", rightSidePose);
        if (isRed) {
            rightSidePose = new Pose2d(
                    FieldMirroringUtils.flip(rightSidePose.getTranslation()),
                    FieldMirroringUtils.flip(rightSidePose.getRotation()));
        }
        Logger.recordOutput("CoralStation/RightSidePoseAdjusted", rightSidePose);
        Transform2d poseDiff =
                robotContainer.driveSimulation.getSimulatedDriveTrainPose().minus(rightSidePose);
        Distance difference =
                Meters.of(Math.sqrt((poseDiff.getX() * poseDiff.getX()) + (poseDiff.getY() * poseDiff.getY())));

        boolean enteredRange = difference.lt(Meters.of(0.75));
        // && Math.abs(angleDiff.in(Degrees)) < 10.0;
        if (enteredRange && !inCoralStationRange) {
            inCoralStationRange = true;
            // right side station not field relative??
            SimulatedArena.getInstance()
                    .addGamePieceProjectile(ReefscapeCoralOnFly.DropFromCoralStation(
                            CoralStationsSide.RIGHT_STATION,
                            DriverStation.getAlliance().get(),
                            false));
        } else if (difference.gt(Meters.of(1.0))) {
            inCoralStationRange = false;
        }
        robotContainer.displaySimFieldToAdvantageScope();
    }
}
