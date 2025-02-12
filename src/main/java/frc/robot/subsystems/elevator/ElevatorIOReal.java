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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.*;

public class ElevatorIOReal implements ElevatorIO {
    final TalonFX leftElevatorMotor = new TalonFX(0, "rio");
    final TalonFX rightElevatorMotor = new TalonFX(1, "rio");
    double elevatorToDistacneRatio = 8.0 / (0.0572958 * Math.PI); // Meters

    private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0.0);

    public ElevatorIOReal() {
        configureMotors();
    }

    @Override
    public void setPosition(Distance targetPosition) {
        leftElevatorMotor.setControl(
                magicRequest.withPosition(targetPosition.in(Meters)).withSlot(0));
        rightElevatorMotor.setControl(
                magicRequest.withPosition(targetPosition.in(Meters)).withSlot(0));
    }

    @Override
    public void stop() {
        leftElevatorMotor.stopMotor();
        rightElevatorMotor.stopMotor();
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.position = Meters.of(leftElevatorMotor.getPosition(true).getValueAsDouble());
    }

    private void configureMotors() {
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
        elevatorMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        elevatorMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.inchesToMeters(52.25);
        elevatorMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        elevatorMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        elevatorMotorConfig.Voltage.PeakForwardVoltage = 10.0;
        elevatorMotorConfig.Voltage.PeakReverseVoltage = -10.0;

        leftElevatorMotor.getConfigurator().apply(elevatorMotorConfig);

        elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightElevatorMotor.getConfigurator().apply(elevatorMotorConfig);

        setPosition(Meters.of(0.0));
    }
}
