// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public interface DriveMap {
    Trigger deployClimber();

    Trigger climbCage();

    Trigger resetClimber();

    Trigger moveToTrough();

    Trigger moveToL2();

    Trigger moveToL3();

    Trigger moveToL4();

    Trigger runIntake();

    Trigger alignRightBranch();

    Trigger alignLeftBranch();

    Trigger resetGyro();

    Trigger zeroElevator();

    BooleanSupplier ejectCoral();
}

class DriverXbox implements DriveMap {
    public final CommandXboxController xboxController;

    protected DriverXbox(int port) {
        this.xboxController = new CommandXboxController(port);
    }

    private Trigger notAligning() {
        return alignRightBranch().negate().and(alignLeftBranch().negate());
    }

    @Override
    public Trigger climbCage() {
        return xboxController.povDown();
    }

    @Override
    public Trigger deployClimber() {
        return xboxController.povUp();
    }

    @Override
    public Trigger resetClimber() {
        return xboxController.povLeft();
    }

    @Override
    public Trigger moveToTrough() {
        return xboxController.b().and(notAligning());
    }

    @Override
    public Trigger moveToL2() {
        return xboxController.a().and(notAligning());
    }

    @Override
    public Trigger moveToL3() {
        return xboxController.x().and(notAligning());
    }

    @Override
    public Trigger moveToL4() {
        return xboxController.y().and(notAligning());
    }

    @Override
    public Trigger alignRightBranch() {
        return xboxController.rightStick();
    }

    @Override
    public Trigger runIntake() {
        return xboxController.rightTrigger(0.3);
    }

    @Override
    public Trigger alignLeftBranch() {
        return xboxController.leftStick();
    }

    @Override
    public Trigger resetGyro() {
        return xboxController.start();
    }

    @Override
    public Trigger zeroElevator() {
        return xboxController.back();
    }

    @Override
    public BooleanSupplier ejectCoral() {
        return xboxController.leftTrigger(0.1);
    }
}
