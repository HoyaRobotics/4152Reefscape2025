// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
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
public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeMotor = new TalonFX(34, "rio");
    private final LaserCan lasercan = new LaserCan(35);
    double intakeRatio = 60.0 / 14;

    private VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private VoltageOut voltageRequest = new VoltageOut(0.0);

    public IntakeIOReal() {
        configureMotors();
        configureLaserCan();
    }

    public boolean hasCoral() {
        Measurement measurement = lasercan.getMeasurement();

        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            double measureDistance = measurement.distance_mm;
            double coralDistance = Inches.of(2).in(Millimeters);

            return measureDistance <= coralDistance;
        } else {
            return false;
        }
    }

    private void configureLaserCan() {
        try {
            lasercan.setRangingMode(RangingMode.SHORT);
            lasercan.setRegionOfInterest(new RegionOfInterest(4, 4, 8, 8));
            lasercan.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    private void configureMotors() {
        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
        intakeMotorConfig.CurrentLimits.StatorCurrentLimit = 40;
        intakeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeMotorConfig.Feedback.SensorToMechanismRatio = intakeRatio;
        intakeMotorConfig.Voltage.PeakForwardVoltage = 11.0;
        intakeMotorConfig.Voltage.PeakReverseVoltage = 11.0;
        intakeMotorConfig.Slot0.kV = 0.5;
        intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        intakeMotor.getConfigurator().apply(intakeMotorConfig);
    }

    @Override
    public void setSpeed(AngularVelocity targetSpeed) {
        intakeMotor.setControl(velocityRequest.withVelocity(targetSpeed).withSlot(0));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        intakeMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
    }

    @Override
    public void setCurrentLimit(Current currentLimit) {
        CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = currentLimit.in(Amp);
        currentLimitConfigs.StatorCurrentLimitEnable = true;

        intakeMotor.getConfigurator().apply(currentLimitConfigs);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        // check sensors for game piece
        inputs.speed = RotationsPerSecond.of(intakeMotor.getVelocity().getValueAsDouble());
        inputs.hasCoral = hasCoral();
    }
}
