// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.*;
import frc.robot.subsystems.arm.ArmConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToLevel extends Command {
    private final Elevator elevator;
    private final Arm arm;
    private final Distance elevatorPosition;
    private final Angle armAngle;

    public MoveToLevel(Elevator elevator, Arm arm, Distance elevatorPosition, Angle armAngle) {
        this.elevator = elevator;
        this.arm = arm;
        this.armAngle = armAngle;
        this.elevatorPosition = elevatorPosition;
        addRequirements(elevator, arm);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevator.setPosition(elevatorPosition.in(Meters));
        arm.setArmPosition(armAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.setPosition(ElevatorConstants.l_Positions.Base.in(Meters));
        arm.setArmPosition(ArmConstants.l_Angles.Base);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
