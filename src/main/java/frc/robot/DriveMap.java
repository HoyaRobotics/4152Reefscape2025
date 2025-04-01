// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public interface DriveMap {
    Trigger deployClimber();

    Trigger climbCage();

    Trigger resetClimber();

    Trigger moveToTrough(boolean aligning);

    Trigger moveToL2(boolean aligning);

    Trigger moveToL3(boolean aligning);

    Trigger moveToL4(boolean aligning);

    Trigger runIntake();

    Trigger runIntakeAlign();

    Trigger scoreProcessor();

    Trigger removeAlgae();

    Trigger scoreBarge();

    Trigger alignRightBranch();

    Trigger alignLeftBranch();

    Trigger resetGyro();

    Trigger zeroElevator();

    DoubleSupplier driveX();

    DoubleSupplier driveY();

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
    public Trigger moveToTrough(boolean aligning) {
        return aligning ? xboxController.b() : xboxController.b().and(notAligning());
    }

    @Override
    public Trigger moveToL2(boolean aligning) {
        return aligning ? xboxController.a() : xboxController.a().and(notAligning());
    }

    @Override
    public Trigger moveToL3(boolean aligning) {
        return aligning ? xboxController.x() : xboxController.x().and(notAligning());
    }

    @Override
    public Trigger moveToL4(boolean aligning) {
        return aligning ? xboxController.y() : xboxController.y().and(notAligning());
    }

    @Override
    public Trigger alignRightBranch() {
        return xboxController.rightStick();
    }

    @Override
    public Trigger alignLeftBranch() {
        return xboxController.leftStick();
    }

    @Override
    public Trigger runIntake() {
        return xboxController.rightTrigger(0.3).and(xboxController.leftTrigger().negate());
    }

    @Override
    public Trigger runIntakeAlign() {
        return xboxController.rightTrigger(0.3).and(xboxController.leftTrigger());
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
    public Trigger scoreBarge() {
        return xboxController.leftBumper();
    }

    @Override
    public Trigger scoreProcessor() {
        return xboxController.leftTrigger().and(xboxController.rightTrigger().negate());
    }

    @Override
    public Trigger removeAlgae() {
        return xboxController.rightBumper();
    }

    @Override
    public BooleanSupplier ejectCoral() {
        return moveToTrough(false);
    }

    @Override
    public DoubleSupplier driveX() {
        return () -> -xboxController.getLeftY();
    }

    @Override
    public DoubleSupplier driveY() {
        return () -> -xboxController.getLeftX();
    }
}
