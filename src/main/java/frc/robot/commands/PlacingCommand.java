// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import java.util.function.BooleanSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlacingCommand extends SequentialCommandGroup {
    /** Creates a new PlacingCommand. */
    public PlacingCommand(
            Elevator elevator,
            Arm arm,
            Intake intake,
            Distance elevatorPosition,
            Angle armPosition,
            BooleanSupplier placeObject) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new MoveToLevel(elevator, arm, elevatorPosition, armPosition),
                new WaitUntilCommand(placeObject),
                IntakeCommands.RunIntakeTimeout(
                        intake, IntakeConstants.IntakeSpeeds.placing, IntakeConstants.PlacingTimeout),
                new ParallelRaceGroup(
                        IntakeCommands.RunIntakeTimeout(intake, IntakeConstants.IntakeSpeeds.placing, 1.0),
                        new MoveToLevel(elevator, arm, ElevatorConstants.l_Positions.Base, ArmConstants.l_Angles.Base)),
                new HoldPosition(elevator, arm, intake));
    }
}
