// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOReal implements ElevatorIO {
    final TalonFX frontElevatorMotor = new TalonFX(30, "rio");
    final TalonFX backElevatorMotor = new TalonFX(31, "rio");
    double elevatorToDistanceRatio = 8.0 / (0.0572958 * Math.PI); // Meters

    private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0.0);

    public ElevatorIOReal() {
        configureMotors();
    }

    @Override
    public void zeroEncoder() {
        frontElevatorMotor.setPosition(0.0);
        backElevatorMotor.setPosition(0.0);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        frontElevatorMotor.setControl(new VoltageOut(voltage));
        backElevatorMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public Current getCurrent() {
        return frontElevatorMotor.getStatorCurrent().getValue();
    }

    @Override
    public void setPosition(Distance targetPosition) {
        frontElevatorMotor.setControl(
                magicRequest.withPosition(targetPosition.in(Meters)).withSlot(0));
        backElevatorMotor.setControl(
                magicRequest.withPosition(targetPosition.in(Meters)).withSlot(0));
    }

    @Override
    public void stop() {
        frontElevatorMotor.stopMotor();
        backElevatorMotor.stopMotor();
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.position = Meters.of(frontElevatorMotor.getPosition(true).getValueAsDouble());
    }

    @Override
    public void changeSoftLimits(boolean enable) {
        SoftwareLimitSwitchConfigs limitSwitchConfig = new SoftwareLimitSwitchConfigs();
        limitSwitchConfig.ForwardSoftLimitEnable = enable;
        limitSwitchConfig.ReverseSoftLimitEnable = enable;
        limitSwitchConfig.ForwardSoftLimitThreshold = Units.inchesToMeters(53.5);
        limitSwitchConfig.ReverseSoftLimitThreshold = 0.0;

        frontElevatorMotor.getConfigurator().apply(limitSwitchConfig);
        backElevatorMotor.getConfigurator().apply(limitSwitchConfig);
    }

    private void configureMotors() {
        TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimit = 35; // 60
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorMotorConfig.Feedback.SensorToMechanismRatio = elevatorToDistanceRatio;
        elevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 2.5;
        elevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 2.0;
        elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorMotorConfig.Slot0.kG = 0.25;
        elevatorMotorConfig.Slot0.kS = 0.28;
        elevatorMotorConfig.Slot0.kV = 5.97;
        elevatorMotorConfig.Slot0.kA = 0.592;
        elevatorMotorConfig.Slot0.kP = 45;
        elevatorMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        elevatorMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.inchesToMeters(53.5);
        elevatorMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        elevatorMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        elevatorMotorConfig.Voltage.PeakForwardVoltage = 10.0;
        elevatorMotorConfig.Voltage.PeakReverseVoltage = -10.0;

        frontElevatorMotor.getConfigurator().apply(elevatorMotorConfig);

        elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        backElevatorMotor.getConfigurator().apply(elevatorMotorConfig);

        frontElevatorMotor.setPosition(0.0);
        backElevatorMotor.setPosition(0.0);

        setPosition(Meters.of(0.0));
    }
}
