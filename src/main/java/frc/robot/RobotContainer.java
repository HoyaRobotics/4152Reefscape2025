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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgaeCommands;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.HoldPosition;
import frc.robot.commands.PlacingCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Side;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeIO;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeIOReal;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeIOSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeAction;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIORealV1;
import frc.robot.subsystems.intake.IntakeIORealV2;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.ArmIO;
import frc.robot.subsystems.superstructure.arm.ArmIOReal;
import frc.robot.subsystems.superstructure.arm.ArmIOSim;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOAdvancedSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
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
    private final Vision vision;
    private final Elevator elevator;
    private final Arm arm;
    private final Intake intake;
    private final AlgaeIntake algaeIntake;
    private final Climber climber;
    private final SuperStructure superStructure;

    public final DriverXbox driveController = new DriverXbox(0);

    public SwerveDriveSimulation driveSimulation = null;

    // Controller
    // public final CommandXboxController driveController = new CommandXboxController(0);

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
                switch (Constants.intakeVersion) {
                    case V1:
                        intake = new Intake(new IntakeIORealV1(), elevator, arm);
                        break;

                    default:
                        intake = new Intake(new IntakeIORealV2(), elevator, arm);
                        break;
                }
                algaeIntake = new AlgaeIntake(new AlgaeIntakeIOReal());
                climber = new Climber(new ClimberIOReal());

                vision = new Vision(drive, new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));

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
                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera0Name,
                                VisionConstants.robotToCamera0,
                                driveSimulation::getSimulatedDriveTrainPose));
                // elevator = new Elevator(new ElevatorIOSim());
                elevator = new Elevator(new ElevatorIOAdvancedSim());
                arm = new Arm(new ArmIOSim(), elevator);
                // arm = new Arm(new ArmIOAdvancedSim(), elevator);
                // intake = new Intake(new IntakeIOSim(driveSimulation, (test) -> {}), elevator, arm);
                intake = new Intake(
                        // 0 angle is horizontal?
                        // -113.5 + 90 + 15 = -8.5
                        // intake is also at a 15 degree angle from the arm
                        new IntakeIOSim(driveSimulation, (targetSpeed) -> {
                            Angle intakeAngle = Degrees.of(arm.getArmPosition().in(Degrees) - 113.5 - 15 + 90);
                            // notes: too much of an angle, not high enough
                            Distance armLength = Inches.of(18.0);
                            Distance intakeY = Inches.of(-1.013);
                            Distance intakeX = armLength.times(-Math.cos(intakeAngle.in(Radians)));
                            Distance intakeZFromCarriage = armLength.times(Math.sin(intakeAngle.in(Radians)));
                            Distance intakeHeight = elevator.getCarriagePose()
                                    .getMeasureZ()
                                    .minus(Inches.of(4.25 / 2))
                                    .plus(Inches.of(16.0))
                                    .plus(intakeZFromCarriage);
                            Translation2d intakePosition = new Translation2d(intakeX, intakeY);
                            // convert from angular to linear velocity?
                            // wheel vs arm radius?
                            LinearVelocity intakeSpeed = MetersPerSecond.of(targetSpeed.in(RadiansPerSecond)
                                    * Inches.of(1.5).in(Meters));
                            SimulatedArena.getInstance()
                                    .addGamePieceProjectile(new ReefscapeCoralOnFly(
                                            driveSimulation
                                                    .getSimulatedDriveTrainPose()
                                                    .getTranslation(),
                                            intakePosition,
                                            driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                            driveSimulation
                                                    .getSimulatedDriveTrainPose()
                                                    .getRotation()
                                                    .rotateBy(new Rotation2d(Degrees.of(180))),
                                            intakeHeight,
                                            intakeSpeed,
                                            intakeAngle)); // 10
                        }),
                        elevator,
                        arm);
                algaeIntake = new AlgaeIntake(new AlgaeIntakeIOSim());
                climber = new Climber(new ClimberIOSim());

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
                vision = new Vision(drive, new VisionIO() {});
                elevator = new Elevator(new ElevatorIO() {});
                arm = new Arm(new ArmIO() {}, elevator);
                intake = new Intake(new IntakeIO() {}, elevator, arm);
                algaeIntake = new AlgaeIntake(new AlgaeIntakeIO() {});
                climber = new Climber(new ClimberIO() {});

                break;
        }

        superStructure = new SuperStructure(elevator, arm);
        intake.setPoseSupplier(superStructure::getTargetPose);

        NamedCommands.registerCommand(
                "alignGetCoralRight",
                Commands.deadline(
                        intake.runWithSensor(IntakeAction.INTAKING)
                                .deadlineFor(superStructure.moveToPose(SuperStructurePose.LOADING))
                                .andThen(() -> intake.stopIntake()),
                        new DriveToPose(drive, () -> CoralStation.getCoralStationPose(Side.RIGHT), Optional.empty())
                                .andThen(Commands.run(
                                        () -> drive.runVelocity(new ChassisSpeeds(0.3, 0.0, 0.0)), drive))));

        NamedCommands.registerCommand(
                "alignGetCoralLeft",
                Commands.deadline(
                        intake.runWithSensor(IntakeAction.INTAKING)
                                .deadlineFor(superStructure.moveToPose(SuperStructurePose.LOADING))
                                .andThen(() -> intake.stopIntake()),
                        new DriveToPose(drive, () -> CoralStation.getCoralStationPose(Side.LEFT), Optional.empty())
                                .andThen(Commands.run(
                                        () -> drive.runVelocity(new ChassisSpeeds(0.3, 0.0, 0.0)), drive))));

        NamedCommands.registerCommand(
                "alignLeftPlaceL4",
                AutoAlign.autoAlignAndPlace(
                        driveController,
                        drive,
                        superStructure,
                        intake,
                        algaeIntake,
                        Side.LEFT,
                        Optional.of(SuperStructurePose.L4),
                        false));
        NamedCommands.registerCommand(
                "alignRightPlaceL4",
                AutoAlign.autoAlignAndPlace(
                        driveController,
                        drive,
                        superStructure,
                        intake,
                        algaeIntake,
                        Side.RIGHT,
                        Optional.of(SuperStructurePose.L4),
                        false));

        NamedCommands.registerCommand(
                "alignRemoveAlgaeV2", AutoAlign.autoAlignAndPickAlgae(drive, superStructure, algaeIntake));
        NamedCommands.registerCommand(
                "alignRemoveAlgaeV1",
                new DriveToPose(drive, () -> Reef.getClosestBranchPose(drive, Side.CENTER), Optional.empty())
                        .andThen(AlgaeCommands.removeL2AlgaeV1(superStructure, intake)));

        NamedCommands.registerCommand("hold", new HoldPosition(elevator, arm, intake, algaeIntake));
        NamedCommands.registerCommand("goToL4", superStructure.moveToPose(SuperStructurePose.L4));

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
                () -> -driveController.xboxController.getLeftY(),
                () -> -driveController.xboxController.getLeftX(),
                () -> -driveController.xboxController.getRightX()));

        elevator.setDefaultCommand(new HoldPosition(elevator, arm, intake, algaeIntake));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.setPose(
                        driveSimulation
                                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
                : () -> drive.setPose(new Pose2d()); // zero gyro
        driveController.resetGyro().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
        driveController.zeroElevator().onTrue(elevator.zeroPosition());

        driveController
                .runIntake()
                .whileTrue(superStructure
                        .moveToPose(SuperStructurePose.LOADING)
                        .alongWith(intake.run(IntakeAction.INTAKING)));

        switch (Constants.intakeVersion) {
            case V1:
                driveController.removeAlgae().onTrue(AlgaeCommands.removeL2AlgaeV1(superStructure, intake));
                break;

            default:
                driveController
                        .removeAlgae()
                        .whileTrue(AutoAlign.autoAlignAndPickAlgae(drive, superStructure, algaeIntake));

                driveController.scoreBarge().whileTrue(AutoAlign.autoScoreBarge(drive, superStructure, algaeIntake));

                driveController
                        .scoreProcessor()
                        .whileTrue(AutoAlign.autoAlignLoadProcessor(drive, superStructure, algaeIntake));
                break;
        }

        /*
        driveController
                .moveToL4(false)
                .onTrue(new PlacingCommand(
                        superStructure, intake, SuperStructurePose.L4, driveController.ejectCoral()));
        driveController
                .moveToL3(false)
                .onTrue(new PlacingCommand(
                        superStructure, intake, SuperStructurePose.L3, driveController.ejectCoral()));
        driveController
                .moveToL2(false)
                .onTrue(new PlacingCommand(
                        superStructure, intake, SuperStructurePose.L2, driveController.ejectCoral()));
                        */
        driveController
                .moveToTrough(false)
                .onTrue(new PlacingCommand(
                        superStructure, intake, SuperStructurePose.TROUGH, driveController.ejectCoral()));

        driveController
                .deployClimber()
                .onTrue(Commands.repeatingSequence(superStructure.moveToPose(SuperStructurePose.CLIMB_STOW))
                        .alongWith(climber.runAngle(ClimberConstants.deployAngle, true)));

        driveController
                .climbCage()
                .onTrue(Commands.repeatingSequence(superStructure.moveToPose(SuperStructurePose.CLIMB_STOW))
                        .alongWith(climber.runAngle(ClimberConstants.climbAngle, false)));

        driveController.resetClimber().onTrue(climber.runAngle(ClimberConstants.baseAngle, true));

        driveController
                .alignLeftBranch()
                .whileTrue(AutoAlign.autoAlignAndPlace(
                        driveController,
                        drive,
                        superStructure,
                        intake,
                        algaeIntake,
                        Side.LEFT,
                        Optional.empty(),
                        true));

        driveController
                .alignRightBranch()
                .whileTrue(AutoAlign.autoAlignAndPlace(
                        driveController,
                        drive,
                        superStructure,
                        intake,
                        algaeIntake,
                        Side.RIGHT,
                        Optional.empty(),
                        true));
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
        intake.addSimulatedGamePiece();
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
