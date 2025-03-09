// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class AlgaeIntakeIOReal implements AlgaeIntakeIO {
    private final TalonFX algaeIntakeMotor = new TalonFX(37, "rio");
    double intakeRatio = 30.0 / 11;

    private VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private VoltageOut voltageRequest = new VoltageOut(0.0);

    public AlgaeIntakeIOReal() {
        configureMotors();
    }

    public boolean hasAlgae() {
        return false;
    }

    private void configureMotors() {
        TalonFXConfiguration algaeIntakeMotorConfig = new TalonFXConfiguration();
        algaeIntakeMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        algaeIntakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        algaeIntakeMotorConfig.Feedback.SensorToMechanismRatio = intakeRatio;
        algaeIntakeMotorConfig.Voltage.PeakForwardVoltage = 11.0;
        algaeIntakeMotorConfig.Voltage.PeakReverseVoltage = -11.0;
        algaeIntakeMotorConfig.Slot0.kV = 0.295;
        algaeIntakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        algaeIntakeMotor.getConfigurator().apply(algaeIntakeMotorConfig);
    }

    @Override
    public void setSpeed(AngularVelocity targetSpeed) {
        algaeIntakeMotor.setControl(velocityRequest.withVelocity(targetSpeed).withSlot(0));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        algaeIntakeMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void stop() {
        algaeIntakeMotor.stopMotor();
    }

    @Override
    public void setCurrentLimit(Current currentLimit) {
        CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = currentLimit.in(Amp);
        currentLimitConfigs.StatorCurrentLimitEnable = true;

        algaeIntakeMotor.getConfigurator().apply(currentLimitConfigs);
    }

    @Override
    public void updateInputs(AlgaeIntakeInputs inputs) {
        // check sensors for game piece
        inputs.speed = RotationsPerSecond.of(algaeIntakeMotor.getVelocity().getValueAsDouble());
        inputs.hasAlgae = hasAlgae();
    }
}
