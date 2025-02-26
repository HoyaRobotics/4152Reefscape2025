// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeAction;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.SuperStructurePose;
import frc.robot.subsystems.superstructure.arm.ArmConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlacingCommand extends SequentialCommandGroup {
    /** Creates a new PlacingCommand. */
    public PlacingCommand(
            SuperStructure superStructure,
            Intake intake,
            SuperStructurePose pose,
            IntakeAction action,
            Angle preArmPosition,
            BooleanSupplier placeObject
            ) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                superStructure.moveToPosePreAngle(pose, preArmPosition),
                new WaitUntilCommand(placeObject),
                intake.run(action).withTimeout(IntakeConstants.PlacingTimeout + IntakeConstants.PostPlacingTimeout)
                        .alongWith(superStructure.retractArm(ArmConstants.l_Angles.Base)));
    }
}
