// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoldPosition extends Command {
    Elevator elevator;
    Arm arm;
    Intake intake;
    /** Creates a new HoldPosition. */
    public HoldPosition(Elevator elevator, Arm arm, Intake intake) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        addRequirements(elevator, arm, intake);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (intake.hasCoral()) {
            elevator.setPosition(ElevatorConstants.l_Positions.Hold);
            arm.setArmPosition(ArmConstants.l_Angles.Hold);
            intake.setSpeed(RevolutionsPerSecond.of(1));
        } else if (!intake.hasCoral()) {
            elevator.setPosition(ElevatorConstants.l_Positions.Base);
            arm.setArmPosition(ArmConstants.l_Angles.Base);
            intake.stopIntake();
            ;
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
