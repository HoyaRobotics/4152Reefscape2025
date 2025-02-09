// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOReal implements ElevatorIO {
    final TalonFX leftElevatorMotor = new TalonFX(0, "rio");
    final TalonFX rightElevatorMotor = new TalonFX(1, "rio");
    double elevatorToDistacneRatio = 8.0 / (0.0572958 * Math.PI); // Meters

    private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0.0);

    @Override
    public void setPosition(double targetPositionMeters) {
        leftElevatorMotor.setControl(
                magicRequest.withPosition(targetPositionMeters).withSlot(0));
        rightElevatorMotor.setControl(
                magicRequest.withPosition(targetPositionMeters).withSlot(0));
    }

    @Override
    public void stop() {
        leftElevatorMotor.stopMotor();
        rightElevatorMotor.stopMotor();
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.positionMeters = leftElevatorMotor.getPosition(true).getValueAsDouble();
    }

    @Override
    public void configureMotors() {
        TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorMotorConfig.Feedback.SensorToMechanismRatio = elevatorToDistacneRatio;
        elevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 0.05;
        elevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.05;
        elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorMotorConfig.Slot0.kG = 0.0;
        elevatorMotorConfig.Slot0.kS = 0.0;
        elevatorMotorConfig.Slot0.kV = 0.0;
        elevatorMotorConfig.Slot0.kA = 0.0;
        elevatorMotorConfig.Slot0.kP = 0.0;
        elevatorMotorConfig.Voltage.PeakForwardVoltage = 10;
        elevatorMotorConfig.Voltage.PeakReverseVoltage = -10;

        leftElevatorMotor.getConfigurator().apply(elevatorMotorConfig);

        elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightElevatorMotor.getConfigurator().apply(elevatorMotorConfig);

        setPosition(0.0);
    }
}
