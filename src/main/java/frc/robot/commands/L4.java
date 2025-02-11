// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L4 extends Command {
    private final Elevator elevator;
    private final Arm arm;
    /** Creates a new L4. */
    public L4(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;
        addRequirements(elevator, arm);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevator.setPosition(Units.inchesToMeters(53.25));
        arm.setArmPosition(Degrees.of(170));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.setPosition(Units.inchesToMeters(0.0));
        arm.setArmPosition(Degrees.of(-49));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
