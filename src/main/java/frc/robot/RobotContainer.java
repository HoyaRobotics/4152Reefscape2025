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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants.Reef.Side;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HoldPosition;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.MoveToLevel;
import frc.robot.commands.PlacingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOAdvancedSim;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import java.util.Arrays;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Elevator elevator;
    private final Arm arm;
    private final Intake intake;

    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                        (pose) -> {});
                elevator = new Elevator(new ElevatorIOReal());
                arm = new Arm(new ArmIOReal(), elevator);
                intake = new Intake(new IntakeIOReal(), elevator, arm);

                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations

                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight, driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);
                elevator = new Elevator(new ElevatorIOSim());
                arm = new Arm(new ArmIOSim(), elevator);
                // arm = new Arm(new ArmIOAdvancedSim(), elevator);
                intake = new Intake(new IntakeIOSim(), elevator, arm);

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                elevator = new Elevator(new ElevatorIO() {});
                arm = new Arm(new ArmIO() {}, elevator);
                intake = new Intake(new IntakeIO() {}, elevator, arm);

                break;
        }

        NamedCommands.registerCommand(
                "intakeReceiveWithSensor",
                IntakeCommands.RunIntakeTillSensed(intake, IntakeConstants.IntakeSpeeds.intaking));
        NamedCommands.registerCommand(
                "intakePlace", IntakeCommands.RunIntakeTimeout(intake, IntakeConstants.IntakeSpeeds.placing, 0.5));
        NamedCommands.registerCommand(
                "intakeReceive", IntakeCommands.RunIntake(intake, IntakeConstants.IntakeSpeeds.intaking));

        NamedCommands.registerCommand(
                "goToL4", new MoveToLevel(elevator, arm, ElevatorConstants.l_Positions.L4, ArmConstants.l_Angles.L4));
        NamedCommands.registerCommand(
                "goToL3", new MoveToLevel(elevator, arm, ElevatorConstants.l_Positions.L3, ArmConstants.l_Angles.L3));
        NamedCommands.registerCommand(
                "goToL2", new MoveToLevel(elevator, arm, ElevatorConstants.l_Positions.L2, ArmConstants.l_Angles.L2));
        NamedCommands.registerCommand(
                "goToTrough",
                new MoveToLevel(elevator, arm, ElevatorConstants.l_Positions.Trough, ArmConstants.l_Angles.Trough));

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureDriverButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureDriverButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.setPose(
                        driveSimulation
                                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
                : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        driverController.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        driverController
                .rightTrigger(0.1)
                .whileTrue(new MoveToLevel(
                                elevator, arm, ElevatorConstants.l_Positions.Loading, ArmConstants.l_Angles.Loading)
                        .alongWith(IntakeCommands.RunIntake(intake, IntakeConstants.IntakeSpeeds.intaking)))
                .onFalse(new HoldPosition(elevator, arm, intake));

        driverController
                .rightBumper()
                .whileTrue(new MoveToLevel(
                                elevator, arm, ElevatorConstants.l_Positions.L2Algae, ArmConstants.l_Angles.L2Algae)
                        .alongWith(IntakeCommands.RunIntake(intake, IntakeConstants.IntakeSpeeds.intaking)))
                .onFalse(new HoldPosition(elevator, arm, intake));

        driverController
                .y()
                .onTrue(new PlacingCommand(
                        elevator,
                        arm,
                        intake,
                        ElevatorConstants.l_Positions.L4,
                        ArmConstants.l_Angles.L4,
                        () -> driverController.leftTrigger(0.1).getAsBoolean(),
                        IntakeConstants.IntakeSpeeds.placing));
        driverController
                .x()
                .onTrue(new PlacingCommand(
                        elevator,
                        arm,
                        intake,
                        ElevatorConstants.l_Positions.L3,
                        ArmConstants.l_Angles.L3,
                        () -> driverController.leftTrigger(0.1).getAsBoolean(),
                        IntakeConstants.IntakeSpeeds.placing));
        driverController
                .a()
                .onTrue(new PlacingCommand(
                        elevator,
                        arm,
                        intake,
                        ElevatorConstants.l_Positions.L2,
                        ArmConstants.l_Angles.L2,
                        () -> driverController.leftTrigger(0.1).getAsBoolean(),
                        IntakeConstants.IntakeSpeeds.placing));
        driverController
                .b()
                .onTrue(new PlacingCommand(
                        elevator,
                        arm,
                        intake,
                        ElevatorConstants.l_Positions.Trough,
                        ArmConstants.l_Angles.Trough,
                        () -> driverController.leftTrigger(0.1).getAsBoolean(),
                        IntakeConstants.IntakeSpeeds.placingTrough));

        driverController.leftStick().onTrue(DriveCommands.driveToPose(drive, () -> {
            Pose2d reefPose = FieldConstants.Reef.offsetReefPose(
                    drive.getPose().nearest(Arrays.asList(FieldConstants.Reef.centerFaces)), Side.LEFT);
            Logger.recordOutput("Reef/PercievedRobot", drive.getPose());
            Logger.recordOutput("Reef/NearestPose", reefPose);
            return reefPose;
        }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void displaySimFieldToAdvantageScope() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
