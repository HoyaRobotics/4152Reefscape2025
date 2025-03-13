// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeIntake.AlgaeIntake;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.AlgaeIntakeAction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoldPosition extends Command {
    private final Elevator elevator;
    private final Arm arm;
    private final Intake intake;
    private final AlgaeIntake algaeIntake;
    /** Creates a new HoldPosition. */
    public HoldPosition(Elevator elevator, Arm arm, Intake intake, AlgaeIntake algaeIntake) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        this.algaeIntake = algaeIntake;
        addRequirements(elevator, arm, intake, algaeIntake);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean hasCoral = intake.hasCoral();
        boolean hasAlgae = algaeIntake.hasAlgae();
        if (hasCoral && hasAlgae) {
            elevator.setPosition(SuperStructurePose.HOLD.elevatorPosition);
            arm.setArmPosition(SuperStructurePose.HOLD.armAngle);
            intake.runIntake(IntakeConstants.IntakeAction.HOLDING);
            algaeIntake.runIntake(AlgaeIntakeAction.HOLDING);
        } else if (hasCoral && !hasAlgae) {
            elevator.setPosition(SuperStructurePose.HOLD.elevatorPosition);
            arm.setArmPosition(SuperStructurePose.HOLD.armAngle);
            intake.runIntake(IntakeConstants.IntakeAction.HOLDING);
            algaeIntake.runIntake(AlgaeIntakeAction.EMPTY);
        } else if (!hasCoral && hasAlgae) {
            elevator.setPosition(SuperStructurePose.HOLD.elevatorPosition);
            arm.setArmPosition(SuperStructurePose.HOLD.armAngle);
            intake.runIntake(IntakeConstants.IntakeAction.EMPTY);
            algaeIntake.runIntake(AlgaeIntakeAction.HOLDING);
        } else if (!hasCoral && !hasAlgae) {
            elevator.setPosition(SuperStructurePose.BASE.elevatorPosition);
            arm.setArmPosition(SuperStructurePose.BASE.armAngle);
            intake.runIntake(IntakeConstants.IntakeAction.EMPTY);
            algaeIntake.runIntake(AlgaeIntakeAction.EMPTY);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
